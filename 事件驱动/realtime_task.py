import threading
import time
import logging
from queue import Queue, Empty
from typing import Dict, List, Optional, Tuple, Set

from bus import CommunicationBus
from motor_pp import PPMotor

logger = logging.getLogger(__name__)


class TaskState:
    IDLE = 'IDLE'
    MOTION_ACTIVE = 'MOTION_ACTIVE'
    STOPPING = 'STOPPING'
    ERROR = 'ERROR'


class RealTimeSyncTask(threading.Thread):
    """类高优先级线程：在收到 SYNC 时被唤醒，并发送下一周期的 PDO。"""

    def __init__(self, bus: CommunicationBus,
                 motors: Dict[int, PPMotor],
                 command_queue: Queue,
                 status_queue: Queue,
                 sync_timeout_s: float = 1.0):
        super().__init__(daemon=True)
        self.bus = bus
        self.motors = motors
        self.command_queue = command_queue
        self.status_queue = status_queue
        self.sync_timeout_s = sync_timeout_s
        self.state = TaskState.IDLE
        self._stop_evt = threading.Event()

        # 轨迹：每个 SYNC 周期的 {node_id -> angle}
        self.current_trajectory = None
        self.trajectory_step = 0
        # 需要在下一个周期将控制字恢复为 0x0F 的电机集合
        self._need_reset_cw_for = set()

    # --- thread control ---
    def stop(self):
        self._stop_evt.set()

    # --- main loop ---
    def run(self):
        logger.info("实时同步任务启动，等待 SYNC 事件...")
        while not self._stop_evt.is_set():
            # 1) 快速处理命令队列（非阻塞）
            self._drain_commands()

            # 2) 等待 SYNC —— 硬实时触发源
            ts = self.bus.wait_for_sync(timeout=self.sync_timeout_s)
            if ts is None:
                logger.warning("等待 SYNC 超时 %.1fs", self.sync_timeout_s)
                continue

            # 3) 每个 SYNC 到来时，按照状态机执行
            try:
                # 优先处理上周期需要复位控制字为 0x0F 的电机（避免持续维持 0x3F）
                if self._need_reset_cw_for:
                    for nid in list(self._need_reset_cw_for):
                        try:
                            self.bus.pdo_set_controlword(nid, 0x0F)
                        except Exception:
                            self.bus.sdo_set_controlword(nid, 0x0F)
                    self._need_reset_cw_for.clear()

                if self.state == TaskState.MOTION_ACTIVE:
                    self._on_sync_motion_active()
                elif self.state == TaskState.IDLE:
                    self._on_sync_idle()
                elif self.state == TaskState.STOPPING:
                    self._on_sync_stopping()
                else:
                    pass
            except Exception as e:
                logger.exception("实时任务异常: %s", e)
                self.state = TaskState.ERROR
                self.status_queue.put({'type': 'error', 'error': str(e)})

    # --- 命令处理 ---
    def _drain_commands(self):
        while True:
            try:
                cmd = self.command_queue.get_nowait()
            except Empty:
                return
            self._handle_command(cmd)

    def _handle_command(self, cmd: dict):
        ctype = cmd.get('type')
        if ctype == 'start_motion':
            self.current_trajectory = cmd['trajectory']  # List[Dict[node_id, angle]]
            self.trajectory_step = 0
            self.state = TaskState.MOTION_ACTIVE
            logger.info("开始轨迹执行，总步数=%d", len(self.current_trajectory))
        elif ctype == 'stop':
            self.state = TaskState.STOPPING
        elif ctype == 'set_angles_immediate':
            # 一次性：在下一个 SYNC 将所有轴设置到指定角度
            self.current_trajectory = [cmd['angles']]
            self.trajectory_step = 0
            self.state = TaskState.MOTION_ACTIVE
        else:
            logger.warning("未知指令: %s", ctype)

    # --- 各状态处理 ---
    def _on_sync_idle(self):
        # 空操作，保持极低开销
        pass

    def _on_sync_motion_active(self):
        if not self.current_trajectory:
            self.state = TaskState.IDLE
            return
        if self.trajectory_step >= len(self.current_trajectory):
            self.status_queue.put({'type': 'motion_finished'})
            self.state = TaskState.IDLE
            self.current_trajectory = None
            return

        angles_for_this_cycle = self.current_trajectory[self.trajectory_step]
        # 将本周期所有电机的指令通过 PDO 发送
        for node_id, angle in angles_for_this_cycle.items():
            motor = self.motors.get(node_id)
            if not motor:
                logger.warning("未找到电机 0x%02X，跳过", node_id)
                continue
            motor.command_angle(angle)
        # 触发新设定点：本周期对涉及的电机发送 0x3F，并在下周期恢复 0x0F
        for node_id in angles_for_this_cycle.keys():
            try:
                self.bus.pdo_set_controlword(node_id, 0x3F)
            except Exception:
                self.bus.sdo_set_controlword(node_id, 0x3F)
            self._need_reset_cw_for.add(node_id)
        self.trajectory_step += 1

    def _on_sync_stopping(self):
    # 简化处理：直接切回空闲。需要时可扩展为减速停止。
        self.state = TaskState.IDLE
        self.status_queue.put({'type': 'stopped'})

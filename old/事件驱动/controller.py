import logging
from queue import Queue
from typing import Dict, List

from realtime_task import RealTimeSyncTask
from motor_pp import PPMotor
from bus import CommunicationBus
from monitor import AngleMonitor

logger = logging.getLogger(__name__)


class SupervisoryController:
    """监督控制器：负责系统初始化、线程管理和对外指令接口。"""

    def __init__(self, bus: CommunicationBus, motors: Dict[int, PPMotor],
                 sync_producer_id: int, sync_period_ms: int = 10):
        self.bus = bus
        self.motors = motors
        self.sync_producer_id = sync_producer_id
        self.sync_period_ms = sync_period_ms

        # 队列与后台线程
        self.command_queue = Queue()
        self.status_queue = Queue()
        self.rt_task = RealTimeSyncTask(bus, motors, self.command_queue, self.status_queue)
        self.angle_monitor = AngleMonitor(motors, period_s=1.0)

    def start(self):
        # 将选定电机配置为 SYNC 生产者
        self.bus.configure_sync_producer(self.sync_producer_id, self.sync_period_ms)
        # 使能所有电机（确保进入 OPERATIONAL）
        for m in self.motors.values():
            # 先清故障，避免进程早期的禁用/错误状态
            try:
                m.clear_fault()
            except Exception:
                pass
            m.basic_init_pp()
            m.enable()
        # 启动实时任务线程
        self.rt_task.start()
        # 启动低频角度监控线程
        self.angle_monitor.start()
        logger.info("监控控制器启动完成")

    def stop(self):
        self.rt_task.stop()
        self.angle_monitor.stop()
        # 此处不强制 join，由外层应用决定

    # --- 高层指令 ---
    def set_angles_next_sync(self, angles: Dict[int, float]):
        """在下一个 SYNC 周期将各节点的角度设置为给定值（单周期）。"""
        self.command_queue.put({'type': 'set_angles_immediate', 'angles': angles})

    def start_trajectory(self, trajectory: List[Dict[int, float]]):
        self.command_queue.put({'type': 'start_motion', 'trajectory': trajectory})

    def stop_motion(self):
        self.command_queue.put({'type': 'stop'})

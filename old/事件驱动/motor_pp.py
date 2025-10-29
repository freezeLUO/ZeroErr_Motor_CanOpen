import time
import logging
from typing import Optional

from bus import CommunicationBus

logger = logging.getLogger(__name__)


class PPMotor:
    """PP（轮廓位置）模式电机辅助类，基于 PDO/SDO。

    仅实现架构所需的关键功能。
    """

    def __init__(self, bus: CommunicationBus, node_id: int, eds_path: Optional[str], period_ms: int):
        self.bus = bus
        self.node_id = node_id
        self.node = bus.add_node(node_id, eds_path)
        self.period_ms = period_ms # ms

    # --- 角度/位置转换 ---
    @staticmethod
    def angle_to_position(angle_deg: float, encoder_resolution: int = 524_288) -> int:
        return int((angle_deg / 360.0) * encoder_resolution)

    @staticmethod
    def position_to_angle(position: int, encoder_resolution: int = 524_288) -> float:
        return (position / encoder_resolution) * 360.0

    # --- 初始化辅助 ---
    def basic_init_pp(self, velocity_rpm: int = 3, accel_rpm2: int = 3, decel_rpm2: int = 3):
        node = self.node
        # 通过 NMT 切换到预操作（可选）
        node.nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.05)

        # 运行模式：PP (0x6060 = 1)
        node.sdo[0x6060].raw = 0x01
        # 轮廓参数
        node.sdo[0x6081].raw = self._rpm_to_pulse(velocity_rpm)
        node.sdo[0x6083].raw = self._rpm_to_pulse(accel_rpm2)
        node.sdo[0x6084].raw = self._rpm_to_pulse(decel_rpm2)

        # 通信周期（可选），需要与 SYNC 周期一致
        node.sdo[0x1006].raw = self.period_ms * 1000  # 转换为微秒

        # 配置最小 PDO：RxPDO1(控制字, 目标位置)；TxPDO1(状态字, 实际位置)
        try:
            txpdo1 = 0x180 + self.node_id
            rxpdo1 = 0x200 + self.node_id
            # 禁用
            node.sdo[0x1800][1].raw = txpdo1 | 0x8000_0000
            node.sdo[0x1400][1].raw = rxpdo1 | 0x8000_0000
            # 传输类型
            node.sdo[0x1800][2].raw = 0x01
            node.sdo[0x1400][2].raw = 0x01
            # 清空映射
            node.sdo[0x1A00][0].raw = 0
            node.sdo[0x1600][0].raw = 0
            # Tx 映射: 0x6041:00:16, 0x6064:00:32
            node.sdo[0x1A00][1].raw = 0x60410010
            node.sdo[0x1A00][2].raw = 0x60640020
            node.sdo[0x1A00][0].raw = 2
            # Rx 映射: 0x6040:00:16, 0x607A:00:32
            node.sdo[0x1600][1].raw = 0x60400010
            node.sdo[0x1600][2].raw = 0x607A0020
            node.sdo[0x1600][0].raw = 2
            # 启用 PDO
            node.sdo[0x1800][1].raw = txpdo1
            node.sdo[0x1400][1].raw = rxpdo1
        except Exception as e:
            logger.warning("节点 0x%02X PDO 配置失败，可能已由 EDS 预配置: %s", self.node_id, e)

        node.nmt.state = 'OPERATIONAL'
        time.sleep(0.05)
        logger.info("节点 0x%02X: 进入 OPERATIONAL，PP 参数已设置", self.node_id)

        # 读取并刷新本地 PDO 映射缓存，确保后续 node.pdo.rx/tx 可用
        try:
            node.pdo.read()
        except Exception as e:
            logger.warning("节点 0x%02X PDO 映射读取失败（将影响 PDO 访问，仍可回退 SDO）: %s", self.node_id, e)

    @staticmethod
    def _rpm_to_pulse(val_rpm: float, encoder_resolution: int = 524_288) -> int:
        # Note: naming kept from user code though units look off; keep compatibility.
        return int((val_rpm / 60.0) * encoder_resolution)

    # --- 使能/禁用 ---
    def enable(self):
        # 先清故障，确保能进入使能流程
        try:
            self.clear_fault()
        except Exception:
            pass
        # 依次执行：Shutdown -> Switch on -> Enable operation
        for cw in (0x06, 0x07, 0x0F):
            try:
                self.bus.pdo_set_controlword(self.node_id, cw)
            except Exception:
                # 如果 PDO 映射不可用则回退 SDO
                logger.warning("PDO 映射不可用，回退到 SDO 写入！")
                self.bus.sdo_set_controlword(self.node_id, cw)
            time.sleep(0.1)
        logger.info("节点 0x%02X: 使能完成", self.node_id)

    def disable(self):
        try:
            self.bus.pdo_set_controlword(self.node_id, 0x00)
        except Exception:
            logger.warning("PDO 映射不可用，回退到 SDO 写入！")
            self.bus.sdo_set_controlword(self.node_id, 0x00)

    # --- 指令 ---
    def command_angle(self, angle_deg: float):
        pos = self.angle_to_position(angle_deg)
        self.bus.pdo_set_target_position(self.node_id, pos)
    # 控制字触发由实时任务统一管理，避免冗余写入

    def get_actual_angle(self) -> float:
        pos = self.bus.sdo_get_actual_position(self.node_id)
        return self.position_to_angle(pos)

    def get_actual_angle_fast(self) -> float:
        """优先通过 TxPDO1 的实际位置读取角度；若不可用则回退 SDO。

        要求 TxPDO1 已映射 0x6064 实际位置，且设备在 SYNC 下周期性发送。
        """
        # 先尝试用索引读取（不依赖 EDS 中文/英文命名差异）
        try:
            pos = int(self.node.pdo.tx[1][0x6064].raw)
            return self.position_to_angle(pos)
        except Exception:
            pass
        # 再尝试常见名称（不同驱动 EDS 可能命名不同）
        for key in (
            'Actual Position',
            'Position actual value',
            'Position Actual Value',
            '实际位置',
        ):
            try:
                pos = int(self.node.pdo.tx[1][key].raw)
                return self.position_to_angle(pos)
            except Exception:
                continue
        logger.warning("TxPDO1 实际位置不可用，回退到 SDO 读取！")
        return self.get_actual_angle()

    # --- 故障处理 ---
    def clear_fault(self):
        """清故障并回到准备就绪状态。"""
        try:
            # 0x80: Fault reset
            self.bus.sdo_set_controlword(self.node_id, 0x80)
            time.sleep(0.05)
        except Exception:
            pass
        try:
            self.bus.sdo_set_controlword(self.node_id, 0x06)
            time.sleep(0.05)
        except Exception:
            pass
        logger.info("节点 0x%02X: 故障清除完成", self.node_id)

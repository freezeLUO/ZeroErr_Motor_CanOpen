import threading
import time
import logging
from typing import Dict, Optional

import canopen


logger = logging.getLogger(__name__)


class SyncMonitor:
    """将接收到的 SYNC 帧 (0x80) 转换成可供等待方使用的事件。"""

    def __init__(self):
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._last_ts: Optional[float] = None

    def on_sync(self, *args):
        """兼容不同 canopen 版本的订阅回调签名。

        可能的签名：
        - (msg,)  → args[0] 为 can.Message
        - (can_id, data, timestamp)  → 3 个位置参数
        我们只需要一个时间戳用于上层节拍，若无则使用本地时间。
        """
        ts = None
        if len(args) == 3:
            # (can_id, data, timestamp)
            ts = args[2]
        # 其他情形统一用本地时间
        if ts is None:
            ts = time.time()
        with self._lock:
            self._last_ts = float(ts)
            self._event.set()

    def wait(self, timeout: Optional[float] = None) -> Optional[float]:
        if not self._event.wait(timeout):
            return None
        # 为下一个周期自动清除
        self._event.clear()
        with self._lock:
            return self._last_ts


class CommunicationBus:
    """对 canopen.Network 的抽象，提供等待 SYNC 与常用辅助函数。"""

    def __init__(self,
                 interface: str = 'pcan',
                 channel: str = 'PCAN_USBBUS1',
                 bitrate: int = 1_000_000):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.network = canopen.Network()
        self._nodes: Dict[int, canopen.RemoteNode] = {}
        self._sync = SyncMonitor()
        self._subscribed = False

    # --- 生命周期 ---
    def connect(self):
        logger.info("连接 CAN 总线: interface=%s, channel=%s, bitrate=%d",
                    self.interface, self.channel, self.bitrate)
        self.network.connect(bustype=self.interface,
                             channel=self.channel,
                             bitrate=self.bitrate)
        if not self._subscribed:
            self.network.subscribe(0x80, self._sync.on_sync)
            self._subscribed = True
            logger.info("已订阅 SYNC (0x80) 报文")

    def disconnect(self):
        try:
            self.network.disconnect()
        finally:
            logger.info("已断开 CAN 总线连接")

    # --- SYNC 同步 ---
    def wait_for_sync(self, timeout: Optional[float] = None) -> Optional[float]:
        return self._sync.wait(timeout)

    # --- 节点 ---
    def add_node(self, node_id: int, eds_path: Optional[str]) -> canopen.RemoteNode:
        node = self.network.add_node(node_id, eds_path)
        self._nodes[node_id] = node
        return node

    def get_node(self, node_id: int) -> Optional[canopen.RemoteNode]:
        return self._nodes.get(node_id)

    # --- SDO 辅助 ---
    def configure_sync_producer(self, node_id: int, period_ms: int):
        """使用 SDO 将指定节点配置为 SYNC 生产者。"""
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")

        # 进入预操作态以进行配置
        node.nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.1)

        # 配置 SYNC 生产者参数
        cobid_value = 0x40000080  # 置位 bit30=1 表示生产者，标准标识符 0x80
        period_us = int(period_ms) * 1000

        logger.info("配置节点 0x%02X 为 SYNC 生产者: 0x1005=0x%08X, 0x1006=%dus",
                    node_id, cobid_value, period_us)

        node.sdo[0x1005].raw = cobid_value
        node.sdo[0x1006].raw = period_us

        # 切回 OPERATIONAL 以实际开始产生 SYNC
        node.nmt.state = 'OPERATIONAL'
        time.sleep(0.1)
        logger.info("节点 0x%02X 已进入 OPERATIONAL，开始广播 SYNC", node_id)

    # --- PP 模式辅助 ---
    @staticmethod
    def _ensure_transmit(pdo):
        try:
            pdo.transmit()
        except Exception:
            # 某些 canopen 版本在设置 .raw 时会自动发送
            pass

    def pdo_set_target_position(self, node_id: int, position: int):
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")
        # 优先按索引写入，避免名称差异
        try:
            try:
                node.pdo.rx[1][0x607A].raw = int(position)
            except Exception:
                node.pdo.rx[1]['Target Position'].raw = int(position)
            self._ensure_transmit(node.pdo.rx[1])
        except Exception:
            # 刷新 PDO 映射后重试一次
            try:
                node.pdo.read()
                try:
                    node.pdo.rx[1][0x607A].raw = int(position)
                except Exception:
                    node.pdo.rx[1]['Target Position'].raw = int(position)
                self._ensure_transmit(node.pdo.rx[1])
                return
            except Exception:
                pass
            # 仍失败则回退 SDO
            logger.warning("PDO 映射不可用，回退到 SDO 写入！")
            node.sdo[0x607A].raw = int(position)

    def pdo_set_controlword(self, node_id: int, cw: int):
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")
        try:
            try:
                node.pdo.rx[1][0x6040].raw = int(cw)
            except Exception:
                node.pdo.rx[1]['Controlword'].raw = int(cw)
            self._ensure_transmit(node.pdo.rx[1])
        except Exception:
            # 刷新 PDO 映射后重试一次
            try:
                node.pdo.read()
                try:
                    node.pdo.rx[1][0x6040].raw = int(cw)
                except Exception:
                    node.pdo.rx[1]['Controlword'].raw = int(cw)
                self._ensure_transmit(node.pdo.rx[1])
                return
            except Exception:
                pass
            # 仍失败则交由上层回退 SDO
            raise

    def sdo_set_controlword(self, node_id: int, cw: int):
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")
        node.sdo[0x6040].raw = int(cw)

    def sdo_get_statusword(self, node_id: int) -> int:
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")
        return int(node.sdo[0x6041].raw)

    def sdo_get_actual_position(self, node_id: int) -> int:
        node = self.get_node(node_id)
        if node is None:
            raise ValueError(f"未找到节点 0x{node_id:02X}")
        return int(node.sdo[0x6064].raw)

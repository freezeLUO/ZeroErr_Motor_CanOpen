import threading
import time
import logging
from typing import Dict

from motor_pp import PPMotor

logger = logging.getLogger(__name__)


class AngleMonitor(threading.Thread):
    """低频角度监控线程：周期性读取并打印所有电机角度，减少对实时任务的干扰。"""

    def __init__(self, motors: Dict[int, PPMotor], period_s: float = 3.0):
        super().__init__(daemon=True)
        self.motors = motors
        self.period_s = period_s
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                angles = {}
                for nid, m in self.motors.items():
                    try:
                        angles[nid] = m.get_actual_angle_fast()
                    except Exception:
                        angles[nid] = None
                if angles:
                    msg = ' '.join(
                        f"{nid:02d}:{angles[nid]:.2f}°" if angles[nid] is not None else f"{nid:02d}:--"
                        for nid in sorted(angles.keys())
                    )
                    logger.info("角度监控 | %s", msg)
            except Exception as e:
                logger.warning("角度监控异常: %s", e)
            time.sleep(self.period_s)

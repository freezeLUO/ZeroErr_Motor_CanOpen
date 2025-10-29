# PP_lib：CiA-402 多模式辅助库

基于 `python-canopen` 的轻量封装，面向 ZeroErr / CiA-402 兼容驱动，覆盖 Profile Position (PP)、Profile Velocity (PV)、Profile Torque (PT) 以及 CSP / CSV / CST 等同步模式的初始化、PDO 映射与高层指令封装。

## 主要特性

- **统一初始化流程**：复位通信、模式确认、402 状态机过度、故障清除与 300 ms 节拍的 enable/disable 保护。
- **PDO 自动配置**：支持 PP/PV/PT/CSP/CSV/CST 的 PDO 重映射与校验，并在映射后刷新 python-canopen 缓存，避免落回 SDO 写入。
- **多模式运动接口**：
  - 位置：`set_target_counts` / `set_target_angle`，自动维持 set-point toggle。
  - 速度：`set_target_velocity_counts` / `set_target_velocity_deg_s`，CSV 模式内支持 SYNC 回调写入。
  - 力矩：`set_target_torque` 与 `get_actual_torque`（SDO 读取 0x6077）。
- **模式切换函数**：`switch_to_profile_velocity_mode`、`switch_to_profile_torque_mode`、`switch_to_cyclic_synchronous_position`、`switch_to_cyclic_synchronous_velocity`、`switch_to_cyclic_synchronous_torque`，均内置 PDO/SDO 配置及安全检查。
- **同步工具**：`SyncProducerHelper` 通过 SDO 配置远程节点成为 SYNC 生产者。
- **CSV 轨迹辅助**：`csv_pd_runner.run_csv_pd_trajectory` 提供基于 PD 的 CSV 轨迹跟踪线程，自动监听 SYNC 帧并写入速度。

## 环境依赖

- Python ≥ 3.10
- `python-canopen`
- `python-can` (底层 CAN 总线驱动)
- 真实 CANopen 设备或 `vcan` 虚拟总线

可选：`matplotlib` 用于 `pp_lib_full_test.py` 绘图演示。

## 快速上手（PP 模式）

```python
import canopen
from motor_controller import PPConfig, ProfilePositionController

network = canopen.Network()
network.connect(bustype="socketcan", channel="can0", bitrate=1_000_000)

cfg = PPConfig(node_id=0x04, sync_period_s=None)
controller = ProfilePositionController(network, cfg)

controller.initialise()
controller.clear_faults()
controller.enable_operation()

controller.set_profile(velocity_deg_s=20.0, accel_deg_s2=50.0, decel_deg_s2=50.0)
controller.set_target_angle(90.0)

print("角度:", controller.get_position_angle())

controller.shutdown()
network.disconnect()
```

根据具体硬件修改总线参数、节点 ID、EDS 路径等。

## 模式切换速查

```python
controller.switch_to_profile_velocity_mode()
controller.set_target_velocity_deg_s(30.0)

controller.switch_to_profile_torque_mode(initial_torque=0)
controller.set_target_torque(150)

controller.switch_to_cyclic_synchronous_velocity(sync_period_ms=20)
controller.set_target_velocity_deg_s(10.0, is_csv=True)

controller.switch_to_cyclic_synchronous_torque(sync_period_ms=20, initial_torque=50)
controller.set_target_torque(80, is_cst=True)

controller.switch_to_cyclic_synchronous_position(sync_period_ms=20)
controller.set_target_angle(45.0)
```

所有切换函数都会：

1. 停机并确保控制器处于 PRE-OP。
2. 重映射 PDO 并验证写入。
3. 预加载零速/零扭矩或当前位置。
4. 选择目标模式并等待 Drive 状态。
5. 清故障并重新启用。

若未显式传入 `sync_period_ms`，函数会尝试使用 `PPConfig.sync_period_s`。

## CSV 模式轨迹执行

`csv_pd_runner.py` 提供 PD 控制示例，兼顾 SYNC 监听与速度命令下发：

```python
from csv_pd_runner import PDGains, run_csv_pd_trajectory

controller.switch_to_cyclic_synchronous_velocity(sync_period_ms=20)

timestamps, planned, actual, cmd_vel, actual_vel = run_csv_pd_trajectory(
    network,
    controller,
    trajectory=[0.0, 1.0, 2.0, ...],
    sample_period_s=0.02,
    gains=PDGains(kp=6.0, kd=0.3, velocity_limit_deg_s=120.0),
)
```

内部会启动线程监听 `0x80` SYNC，自动处理异常并在结束时发送 halt。

## SYNC 生产者配置

```python
from motor_controller import SyncProducerHelper

sync_helper = SyncProducerHelper(network)
sync_helper.enable_sync_producer(node_id=0x04, period_ms=20)

# 停止时
sync_helper.disable_sync_producer(node_id=0x04)
```

调用前需确保网络对象已添加目标节点。

## 日志建议

库默认使用模块级 `log = logging.getLogger(__name__)`。若想屏蔽 python-canopen 在读取 PDO 配置时的冗余 INFO，可在上层应用调整：

```python
import logging

logging.basicConfig(level=logging.INFO)
logging.getLogger("canopen.pdo").setLevel(logging.WARNING)
```

## 示例脚本

- `pp_lib_full_test.py`：从 PP→PT→CSV 等模式的端到端演示，含轨迹规划与绘图。
- `csv_pd_runner.py`：独立的 CSV 回调辅助。

运行示例前请确认：

1. `ZeroErr Driver_V1.5.eds` 可从当前目录访问。
2. 目标驱动支持通过 SDO 写入 PDO 映射。
3. 网络参数（`bustype` / `channel` / `bitrate`）与实际硬件匹配。


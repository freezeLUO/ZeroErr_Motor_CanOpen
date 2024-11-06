import canopen
import time
import logging
import can
import threading
import numpy as np
from scipy.interpolate import make_interp_spline

# 配置日志记录
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Motor_CSP:
    def __init__(self, node_id, network, encoder_resolution=524288):
        self.node_id = node_id
        self.network = network
        self.encoder_resolution = encoder_resolution
        self.motor = self.network.add_node(self.node_id, "ZeroErr Driver_V1.5.eds")
        self.motor.load_configuration()  # 加载配置，防止覆盖自定义PDO映射
        self.initialize_node()  # 初始化节点
        self.setup_pdo_for_csp()  # 配置PDO映射
        time.sleep(0.1)
        self.start_node()  # 启动节点
        time.sleep(0.1)
        self.clear_fault()  # 清除故障
        time.sleep(0.1)
        self.enable_motor()  # 电机使能
        time.sleep(0.1)


    def angle_to_position(self, angle):
        """将角度转换为位置脉冲值"""
        position = (angle / 360) * self.encoder_resolution
        return int(position)

    def position_to_angle(self, position):
        """将位置脉冲值转换为角度"""
        angle = (position / self.encoder_resolution) * 360
        return angle
    
    def velocity_to_pulse(self, velocity_rpm):
        """将速度转换为脉冲值"""
        velocity_pulse_per_sec = (velocity_rpm / 60) * self.encoder_resolution
        return int(velocity_pulse_per_sec)

    def acceleration_to_pulse(self, acceleration_rpm2):
        """将加速度转换为脉冲值"""
        acceleration_pulse_per_sec2 = (acceleration_rpm2 / 60) * self.encoder_resolution
        return int(acceleration_pulse_per_sec2)

    def clear_fault(self):
        """清除电机的故障状���"""
        logger.info(f"电机 {self.node_id}: 清除故障")
        self.set_controlword(0x0080)  # 发送复位故障命令
        time.sleep(0.1)
        self.set_controlword(0x0006)  # 进入准备就绪状态
        time.sleep(0.1)

    def initialize_node(self):
        """初始化节点"""
        # 停止远程节点
        message = can.Message(
            arbitration_id=0x000,      # NMT 指令的 COB-ID 是 0x000
            data=[0x02, self.node_id], # 0x02 是 Stop remote node，第二字节是节点 ID
            is_extended_id=False
        )
        self.network.bus.send(message)
        logger.info(f"电机 {self.node_id}:停止远程节点指令发送完成")

        # 复位节点
        message = can.Message(
            arbitration_id=0x000,      # NMT 指令的 COB-ID 是 0x000
            data=[0x82, self.node_id], # 0x82 是 Reset communication，第二字节是节点 ID
            is_extended_id=False
        )
        self.network.bus.send(message)
        logger.info(f"电机 {self.node_id}:复位节点指令发送完成")

        # 设置为CSP模式
        self.motor.sdo[0x6060].raw = 0x08  # 发送 2F 60 60 00 01 00 00 00
        logger.info(f"电机 {self.node_id}:周期位置同步模式设置完成")

        # 核对运行模式为 CSP 模式
        if self.motor.sdo[0x6061].raw == 0x08:  # 发送 40 61 60 00 00 00 00 00
            logger.info(f"电机 {self.node_id}:运行模式为 CSP 模式")
        else:
            logger.warning(f"电机 {self.node_id}:运行模式非 CSP 模式，请检查配置")
        time.sleep(0.1)

        # 关闭同步发生器
        self.motor.sdo[0x1006].raw = 0  # 发送 23 05 10 00 00 00 00 00
        logger.info(f"电机 {self.node_id}:同步发生器已关闭")

        # 通信周期设置为 1000us
        self.motor.sdo[0x1006].raw = 1000  # 发送 23 06 10 00 E8 03 00 00
        logger.info(f"电机 {self.node_id}:通信周期设置为 1000us")

    def setup_pdo_for_csp(self):
        """配置RxPDO和TxPDO映射"""
        self.motor.nmt.state = 'PRE-OPERATIONAL'

        # 配置RxPDO1：映射控制字和目标位置
        self.motor.sdo[0x1400][1].raw = 0x80000200 + self.node_id  # 禁用RxPDO1
        self.motor.sdo[0x1600][0].raw = 0  # 清空映射条目
        self.motor.sdo[0x1600][1].raw = 0x60400010  # 控制字（16位）
        self.motor.sdo[0x1600][2].raw = 0x607A0020  # 目标位置（32位）
        self.motor.sdo[0x1600][0].raw = 2  # 映射两个对象
        self.motor.sdo[0x1400][1].raw = 0x00000200 + self.node_id  # 启用RxPDO1
        #self.motor.sdo[0x1400][2].raw = 0x01  

        # 配置TxPDO1：映射状态字和实际位置
        self.motor.sdo[0x1800][1].raw = 0x80000180 + self.node_id  # 禁用TxPDO1
        self.motor.sdo[0x1A00][0].raw = 0  # 清空映射条目
        self.motor.sdo[0x1A00][1].raw = 0x60410010  # 状态字（16位）
        self.motor.sdo[0x1A00][2].raw = 0x60640020  # 实际位置（32位）
        self.motor.sdo[0x1A00][0].raw = 2  # 映射两个对象
        self.motor.sdo[0x1800][1].raw = 0x00000180 + self.node_id  # 启用TxPDO1
       #self.motor.sdo[0x1800][2].raw = 0x01  

        # 启用PDO通信
        self.motor.pdo.rx[1].enabled = True
        self.motor.pdo.tx[1].enabled = True

        self.motor.nmt.state = 'OPERATIONAL'  # 切换到操作模式

        logger.info(f"电机 {self.node_id} PDO映射配置完成")


    def start_node(self):
        """启动节点"""
        self.network.nmt.send_command(0x01)  # NMT启动节点
        logger.info(f"电机 {self.node_id}:NMT启动远程节点完成")
        # 获取实际位置
        actual_position = self.motor.sdo[0x6064].raw  # 0x6064 是实际位置的对象字典索引
        # 写入实际位置
        self.motor.sdo[0x607A].raw = actual_position
        actual_position = self.position_to_angle(actual_position)
        logger.info(f"实际位置：{round(actual_position, 2)}°")


    def send_sync_frame(self):
        """发送同步帧"""
        sync_message = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
        self.network.bus.send(sync_message)
        logger.debug(f"电机 {self.node_id}:同步帧发送完成")
        
    def send_controlword(self, controlword):
        """通过PDO发送控制字"""
        self.motor.pdo.rx[1]['Controlword'].raw = controlword
        self.motor.pdo.rx[1].transmit()
        self.send_sync_frame()

    def check_statusword(self, expected_value, mask, description):
        """检查状态字是否符合预期值"""
        for _ in range(50):  # 最多等待5秒钟
            statusword = self.motor.pdo.tx[1]['Statusword'].raw
            if (statusword & mask) == expected_value:
                logger.info(f"电机 {self.node_id}: 进入 {description} 状态")
                return
            time.sleep(0.1)
        raise Exception(f"电机 {self.node_id}: 未能进入 {description} 状态，当前状态字：0x{statusword:04X}")

    def enable_motor(self):
        """电机使能过程，使用PDO进行控制"""
        # 步骤：Shutdown -> Switch On -> Operation Enabled
        self.send_controlword(0x0006)  # Shutdown
        time.sleep(0.1)
        self.check_statusword(0x0021, 0x006F, "Ready to Switch On")

        self.send_controlword(0x0007)  # Switch On
        time.sleep(0.1)
        #self.check_statusword(0x0023, 0x006F, "Switched On")

        self.send_controlword(0x000F)  # Operation Enabled
        time.sleep(0.1)
        #self.check_statusword(0x0027, 0x006F, "Operation Enabled")

        
        logger.info(f"电机 {self.node_id}:电机使能完成")

    def check_target_reached(self):
        """监控目标位置是否到达"""
        #statusword = self.motor.pdo.tx[1]['Statusword'].raw
        statusword = self.motor.sdo[0x6041].raw  # 读取 Statusword (0x6041)
        return (statusword & 0x0400) != 0  # 检查Statusword的第10位


    def get_actual_position(self):
        """获取电机的当前实际位置，通过PDO读取"""
        actual_position = self.motor.pdo.tx[1]['Position actual value'].raw
        logger.debug(f"电机 {self.node_id}: 当前实际位置为 {actual_position} plus")
        return actual_position
    
    def set_controlword(self, controlword):
        """设置控制字到 0x6040"""
        self.motor.pdo.rx[1]['Controlword'].raw = controlword
        self.motor.pdo.rx[1].transmit()
        logger.debug(f"电机 {self.node_id}:通过 PDO 发送控制字: {hex(controlword)}")

    def go_to_position(self, position):
        """移动电机到指定角度"""
        self.motor.pdo.rx[1]['Target Position'].raw = position
        self.motor.pdo.rx[1].transmit()
        #self.send_sync_frame()

def main():
    network = canopen.Network()
    network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    def GO(traj, motor,interval=0.01):
        for position in traj:
            # 记录开始时间
            start_time = time.perf_counter()           
            # 更新目标位置
            motor.go_to_position(position)
            #motor2.go_to_position(position)
            motor.send_sync_frame()         
            # 计算需要等待的时间
            elapsed_time = time.perf_counter() - start_time
            sleep_time = max(0, interval - elapsed_time)          
            # 保证精确的间隔时间
            time.sleep(sleep_time)

    try:
        motor1 = Motor_CSP(0x04, network)
        time.sleep(0.5)
        #motor2 = Motor_CSP(0x03, network)
        # 设置控制点
        x_points = np.array([0, 500, 1000])  # X轴上的控制点
        y_points = np.array([0, 180, 0])  # Y轴上的控制点
        # 创建三次样条曲线对象
        spline = make_interp_spline(x_points, y_points, k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))  # k=3表示三次样条
        # 生成 1000 个均匀分布的 x 值
        x_1000 = np.linspace(0, 1000, 1000)
        y_1000 = spline(x_1000)  # 计算每个 x 值对应的 y 值
        # 将 y 值存入数组
        y_values_array = y_1000
        target_positions = [motor1.angle_to_position(angle) for angle in y_values_array]

        # 设置控制点
        x_points1 = np.array([0, 1000, 2000])  # X轴上的控制点
        y_points1 = np.array([45, 225, 45])  # Y轴上的控制点
        # 创建三次样条曲线对象
        spline1 = make_interp_spline(x_points1, y_points1, k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))  # k=3表示三次样条
        # 生成 1000 个均匀分布的 x 值
        x_10001 = np.linspace(0, 2000, 2000)
        y_10001 = spline1(x_10001)  # 计算每个 x 值对应的 y 值
        # 将 y 值存入数组
        y_values_array1 = y_10001
        target_positions1 = [motor1.angle_to_position(angle) for angle in y_values_array1]
        interval = 0.01  # 10毫秒
        # arry1 = np.linspace(0, 45, 500)
        # target_positions_init = [motor1.angle_to_position(angle) for angle in arry1]
        time.sleep(1)
        motor1.send_sync_frame()
        actuall_position = motor1.get_actual_position()
        motor1.go_to_position(actuall_position)
        motor1.send_sync_frame()
        time.sleep(1)
        
        GO(target_positions, motor1)

        logger.info("CSP轨迹跟随完成")
        crrent_position = motor1.get_actual_position()
        crrent_position = motor1.position_to_angle(crrent_position)
        logger.info(f"当前位置：{crrent_position}")

    finally:

        network.disconnect()
        logger.info("已断开与CAN网络的连接")


if __name__ == "__main__":
    main()
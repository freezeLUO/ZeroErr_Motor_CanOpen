import canopen
import time
import logging
import can
import threading

# 配置日志记录
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MotorAngleReader:
    def __init__(self, motors, utils,interval=1.0):
        """
        初始化 MotorAngleReader 类
        :param motors: 需要读取角度的电机列表
        :param interval: 读取角度的时间间隔（秒）
        """
        self.motors = motors
        self.interval = interval
        self.running = False
        self.thread = None
        self.utils = utils

    def start(self):
        """启动读取角度的线程"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_angles)
            self.thread.start()

    def stop(self):
        """停止读取角度的线程"""
        if self.running:
            self.running = False
            if self.thread is not None:
                self.thread.join()

    def _read_angles(self):
        """定时读取各个电机的角度"""
        while self.running:
            for motor in self.motors:
                actual_position = motor.motor.sdo[0x6064].raw  # 读取实际位置
                angle = self.utils.position_to_angle(actual_position)  # 转换为角度
                logger.debug(f"电机 {motor.node_id}: 当前角度为 {angle:.2f}°")
            time.sleep(self.interval)

class MotorUtils:
    def __init__(self, encoder_resolution=524288):
        self.encoder_resolution = encoder_resolution

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

class Motor_PP:
    def __init__(self, node_id, network, encoder_resolution=524288):
        self.node_id = node_id
        self.network = network
        self.encoder_resolution = encoder_resolution
        self.motor = self.network.add_node(self.node_id, "ZeroErr Driver_V1.5.eds")
        self.motor.load_configuration()  # 加载配置，防止覆盖自定义PDO映射
        self.initialize_node()  # 初始化节点
        self.configure_pdo()  # 配置PDO映射
        time.sleep(0.1)
        self.start_node()  # 启动节点
        self.set_immediate_effect(True)  # 设置立即生效
        self.clear_fault()  # 清除故障
        self.enable_motor()  # 电机使能

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

        # 设置为轮廓位置模式
        self.motor.sdo[0x6060].raw = 0x01  # 发送 2F 60 60 00 01 00 00 00
        logger.info(f"电机 {self.node_id}:轮廓位置模式设置完成")

        # 核对运行模式为 PP 模式
        if self.motor.sdo[0x6061].raw == 0x01:  # 发送 40 61 60 00 00 00 00 00
            logger.info(f"电机 {self.node_id}:运行模式为 PP 模式")
        else:
            logger.warning(f"电机 {self.node_id}:运行模式非 PP 模式，请检查配置")

        # 设置轮廓速度为 20°/s
        self.motor.sdo[0x6081].raw = self.velocity_to_pulse(5)
        logger.info(f"电机 {self.node_id}:轮廓速度设置为 10°/s")

        # 设置轮廓加速度为 20°/s²
        self.motor.sdo[0x6083].raw = self.acceleration_to_pulse(5)
        logger.info(f"电机 {self.node_id}:轮廓加速度设置为 10°/s²")

        # 设置轮廓减速度为 20°/s²
        self.motor.sdo[0x6084].raw = self.acceleration_to_pulse(5)
        logger.info(f"电机 {self.node_id}:轮廓减速度设置为 10°/s²")

        # 关闭同步发生器
        self.motor.sdo[0x1006].raw = 0  # 发送 23 05 10 00 00 00 00 00
        logger.info(f"电机 {self.node_id}:同步发生器已关闭")

        # 通信周期设置为 1000us
        self.motor.sdo[0x1006].raw = 1000  # 发送 23 06 10 00 E8 03 00 00
        logger.info(f"电机 {self.node_id}:通信周期设置为 1000us")

    def set_profile_velocity(self, velocity_rpm):
        """设置轮廓速度"""
        self.motor.sdo[0x6081].raw = self.velocity_to_pulse(velocity_rpm)
        logger.info(f"电机 {self.node_id}: 轮廓速度设置为 {velocity_rpm}°/s")

    def set_profile_acceleration(self, acceleration_rpm2):
        """设置轮廓加速度"""
        self.motor.sdo[0x6083].raw = self.acceleration_to_pulse(acceleration_rpm2)
        logger.info(f"电机 {self.node_id}: 轮廓加速度设置为 {acceleration_rpm2}°/s²")

    def set_profile_deceleration(self, deceleration_rpm2):
        """设置轮廓减速度"""
        self.motor.sdo[0x6084].raw = self.acceleration_to_pulse(deceleration_rpm2)
        logger.info(f"电机 {self.node_id}: 轮廓减速度设置为 {deceleration_rpm2}°/s²")

    def set_profile_parameters(self, velocity_rpm, acceleration_rpm2, deceleration_rpm2):
        """一起设置轮廓速度、加速度和减速度"""
        self.set_profile_velocity(velocity_rpm)
        self.set_profile_acceleration(acceleration_rpm2)
        self.set_profile_deceleration(deceleration_rpm2)
        logger.info(f"电机 {self.node_id}: 轮廓参数设置完成 - 速度: {velocity_rpm}°/s, 加速度: {acceleration_rpm2}°/s², 减速度: {deceleration_rpm2}°/s²")

    def configure_pdo(self):
        """根据官方手册配置 TxPDO1 和 RxPDO1 的映射"""
        
        # 计算基于 node_id 的 COB-ID
        txpdo1_cob_id = 0x180 + self.node_id
        rxpdo1_cob_id = 0x200 + self.node_id
        self.motor.nmt.state = 'PRE-OPERATIONAL'
        logger.debug(f"电机 {self.node_id}:进入 PRE-OPERATIONAL 状态")

        # 配置 TxPDO1
        logger.info(f"电机 {self.node_id}:开始配置 TxPDO1")
        
        # 1. 禁用 TxPDO1
        self.motor.sdo[0x1800][1].raw = txpdo1_cob_id | 0x80000000  # COB-ID + 禁用位
        logger.debug(f"电机 {self.node_id}:关闭 TxPDO1")

        # 2. 设置传输类型
        self.motor.sdo[0x1800][2].raw = 0x01  # 异步传输
        logger.debug(f"电机 {self.node_id}:设置 TxPDO1 传输类型为异步传输 (0x01)")

        # 3. 清除 TxPDO1 映射
        self.motor.sdo[0x1A00][0].raw = 0x00
        logger.debug(f"电机 {self.node_id}:清除 TxPDO1 原有映射")

        # 4. 设置映射对象：状态字 (Status Word)
        self.motor.sdo[0x1A00][1].raw = 0x60410010
        logger.debug(f"电机 {self.node_id}:映射状态字 (Status Word) 到 TxPDO1")

        # 5. 设置映射对象：实际位置 (Actual Position)
        self.motor.sdo[0x1A00][2].raw = 0x60640020
        logger.debug(f"电机 {self.node_id}:映射实际位置 (Actual Position) 到 TxPDO1")

        # 6. 设置 TxPDO1 映射对象数量为 2
        self.motor.sdo[0x1A00][0].raw = 0x02
        logger.debug(f"电机 {self.node_id}:TxPDO1 映射对象个数设置为 2")

        # 7. 设置传输类型并启用 TxPDO1
        self.motor.sdo[0x1800][2].raw = 0xFF  # 异步传输类型
        self.motor.sdo[0x1800][1].raw = txpdo1_cob_id  # 去掉禁用位，设置 COB-ID
        logger.debug(f"电机 {self.node_id}:开启 TxPDO1 并设置传输类型为异步 (0xFF)")

        # 配置 RxPDO1
        logger.info(f"电机 {self.node_id}:开始配置 RxPDO1")

        # 1. 禁用 RxPDO1
        self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id | 0x80000000  # COB-ID + 禁用位
        logger.debug(f"电机 {self.node_id}:关闭 RxPDO1")

        # 2. 设置传输类型
        self.motor.sdo[0x1400][2].raw = 0x01  # 异步传输
        logger.debug(f"电机 {self.node_id}:设置 RxPDO1 传输类型为异步传输 (0x01)")

        # 3. 清除 RxPDO1 映射
        self.motor.sdo[0x1600][0].raw = 0x00
        logger.debug(f"电机 {self.node_id}:清除 RxPDO1 原有映射")

        # 4. 设置映射对象：控制字 (Control Word)
        self.motor.sdo[0x1600][1].raw = 0x60400010
        logger.debug(f"电机 {self.node_id}:映射控制字 (Control Word) 到 RxPDO1")

        # 5. 设置映射对象：目标位置 (Target Position)
        self.motor.sdo[0x1600][2].raw = 0x607A0020
        logger.debug(f"电机 {self.node_id}:映射目标位置 (Target Position) 到 RxPDO1")

        # 6. 设置 RxPDO1 映射对象数量为 2
        self.motor.sdo[0x1600][0].raw = 0x02
        logger.debug(f"电机 {self.node_id}:RxPDO1 映射对象个数设置为 2")

        # 7. 设置传输类型并启用 RxPDO1
        self.motor.sdo[0x1400][2].raw = 0xFF  # 异步传输类型
        self.motor.sdo[0x1400][1].raw = rxpdo1_cob_id  # 去掉禁用位，设置 COB-ID
        logger.debug(f"电机 {self.node_id}:开启 RxPDO1 并设置传输类型为异步 (0xFF)")

        logger.info(f"电机 {self.node_id}:PDO 配置完成")
    
    def start_node(self):
        """启动节点"""
        self.network.nmt.send_command(0x01)  # NMT启动节点
        logger.info(f"电机 {self.node_id}:NMT启动远程节点完成")
        # 获取实际位置
        actual_position = self.motor.sdo[0x6064].raw  # 0x6064 是实际位置的对象字典索引
        actual_position = self.position_to_angle(actual_position)
        logger.info(f"实际位置：{round(actual_position, 2)}°")
        # 发送同步帧
        self.send_sync_frame()

    def set_immediate_effect(self, immediate):
        """设置立即生效或非立即生效"""
        controlword = self.motor.sdo[0x6040].raw
        if immediate:
            controlword |= (1 << 5)  # 设置Bit 5为1（立即生效）
            logger.info(f"电机 {self.node_id}:设置为立即生效")
        else:
            controlword &= ~(1 << 5)  # 设置Bit 5为0（非立即生效）
            logger.info(f"电机 {self.node_id}:设置为非立即生效")
        self.motor.sdo[0x6040].raw = controlword
        logger.debug(f"电机 {self.node_id}:控制字已更新为: {hex(controlword)}")

    def send_sync_frame(self):
        """发送同步帧"""
        sync_message = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
        self.network.bus.send(sync_message)
        logger.debug(f"电机 {self.node_id}:同步帧发送完成")

    def enable_motor(self):
        """电机使能过程，使用PDO进行控制"""
        self.set_controlword(0x06)  # Shutdown (关闭)
        self.send_sync_frame()
        time.sleep(0.1)
        self.set_controlword(0x07)  # Switch on (准备使能)
        self.send_sync_frame()
        time.sleep(0.1)
        self.set_controlword(0x0F)  # Enable operation (使能)
        self.send_sync_frame()
        time.sleep(0.1)
        logger.info(f"电机 {self.node_id}:电机使能完成")

    def check_target_reached(self):
        """监控目标位置是否到达"""
        #statusword = self.motor.pdo.tx[1]['Statusword'].raw
        statusword = self.motor.sdo[0x6041].raw  # 读取 Statusword (0x6041)
        return (statusword & 0x0400) != 0  # 检查Statusword的第10位

    def monitor_target_reached(self, timeout=10):
        """监控电机是否到达目标位置"""
        start_time = time.time()
        while self.monitoring.is_set():
            if self.check_target_reached():
                actual_position = self.motor.sdo[0x6064].raw
                logger.info(f"电机 {self.node_id}: 已到达目标位置, 当前实际位置: {actual_position}")
                self.monitoring.clear()  # 停止监控
                break
            if time.time() - start_time > timeout:
                logger.warning(f"电机 {self.node_id}: 未能在 {timeout} 秒内到达目标位置")
                self.monitoring.clear()
                break
            logger.info(f"电机 {self.node_id}: 目标未到达。当前位置: {self.get_actual_position()}")
            time.sleep(0.1)

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

    def go_to_position(self, angle):
        """移动电机到指定角度"""
        position = self.angle_to_position(angle)
        self.target_position = position
        self.tracking_target = True
        self.motor.pdo.rx[1]['Target Position'].raw = position  # 设置目标位置
        self.send_sync_frame()
        self.set_controlword(0x0F)  # Enable operation (使能)
        self.send_sync_frame()
        self.set_controlword(0x3F)  # Command triggered (命令触发)
        self.send_sync_frame()


def main():
    """主程序，实例化多个电机并控制它们"""
    network = canopen.Network()
    network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    Utils = MotorUtils()
    try:
        motor = Motor_PP(0x02, network)
        AngleMonitor = MotorAngleReader([motor], Utils, interval=1.0)
        AngleMonitor.start()
        while True:
            angle_input = input("请输入角度值 (按回车执行, 输入 'q' 退出, 输入 'p' 获取当前位置): ")
            if angle_input.lower() == 'q':
                break
            elif angle_input.lower() == 'p':
                current_position = motor.get_actual_position()
                current_angle = Utils.position_to_angle(current_position)
                print(f"电机 {motor.node_id}: 当前位置: {round(current_angle, 4)}°")
            else:
                try:
                    angle = float(angle_input)
                    motor.go_to_position(angle)
                except ValueError:
                    logger.error("请输入有效的数字")
                except Exception as e:
                    logger.error(f"发生错误: {e}")

    finally:
        AngleMonitor.stop()
        network.disconnect()
        logger.info("已断开与CAN网络的连接")

if __name__ == "__main__":
    main()
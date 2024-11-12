import canopen
import time
import logging
import threading
import tkinter as tk
from tkinter import ttk
from MultiMotorControl_PP import *

# 配置日志记录
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MotorControlGUI:
    def __init__(self, root, motors, utils):
        self.root = root
        self.motors = motors
        self.utils = utils
        self.angle_vars = {}
        self.angle_labels = {}
        self.create_widgets()
        self.create_angle_reader()
    
    def create_widgets(self):
        """创建GUI控件"""
        self.root.title("机械臂上位机示教器")
        mainframe = ttk.Frame(self.root, padding="10")
        mainframe.grid(column=0, row=0, sticky='NSEW')
        
        for idx, motor in enumerate(self.motors):
            # 角度变量
            angle_var = tk.DoubleVar()
            actual_position = motor.get_actual_position()
            actual_angle = self.utils.position_to_angle(actual_position)
            angle_var.set(actual_angle)
            self.angle_vars[motor.node_id] = angle_var
            
            # 电机标签
            ttk.Label(mainframe, text=f"电机 {motor.node_id}").grid(column=0, row=idx, sticky='W')
            
            # 滑块
            slider = ttk.Scale(mainframe, from_=0, to=360, orient='horizontal', variable=angle_var, command=lambda val, m=motor: self.update_motor_angle(m, val))
            slider.grid(column=1, row=idx, sticky='EW')
            mainframe.columnconfigure(1, weight=1)
            
            # 实际角度显示
            angle_label = ttk.Label(mainframe, text=f"当前角度: {actual_angle:.2f}°")
            angle_label.grid(column=2, row=idx, sticky='E')
            self.angle_labels[motor.node_id] = angle_label
            
            # 精确角度输入框
            angle_entry = ttk.Entry(mainframe, textvariable=angle_var, width=10)
            angle_entry.grid(column=3, row=idx, sticky='E')
            angle_entry.bind("<Return>", lambda event, m=motor, v=angle_var: self.update_motor_angle(m, v.get()))
            
            # 确认按钮
            confirm_button = ttk.Button(mainframe, text="确认", command=lambda m=motor, v=angle_var: self.update_motor_angle(m, v.get()))
            confirm_button.grid(column=4, row=idx, sticky='E')
            
            # 添加jog模式按钮
            jog_forward_button = ttk.Button(mainframe, text="正转5°", command=lambda m=motor: self.jog_motor(m, 5))
            jog_forward_button.grid(column=5, row=idx, sticky='E')
            
            jog_backward_button = ttk.Button(mainframe, text="反转5°", command=lambda m=motor: self.jog_motor(m, -5))
            jog_backward_button.grid(column=6, row=idx, sticky='E')
            
        # 窗口尺寸调整
        for child in mainframe.winfo_children():
            child.grid_configure(padx=5, pady=5)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
    
    def update_motor_angle(self, motor, val):
        """更新电机目标角度"""
        try:
            angle = float(val)
            motor.go_to_position(angle)
        except Exception as e:
            logger.error(f"更新电机 {motor.node_id} 角度时出错: {e}")
    
    def jog_motor(self, motor, delta_angle):
        """使电机转动指定的角度增量"""
        try:
            actual_position = motor.get_actual_position()
            actual_angle = self.utils.position_to_angle(actual_position)
            target_angle = actual_angle + delta_angle
            motor.go_to_position(target_angle)
        except Exception as e:
            logger.error(f"Jog电机 {motor.node_id} 时出错: {e}")
    
    def create_angle_reader(self):
        """创建电机角度读取线程"""
        self.angle_reader = MotorAngleReader(self.motors, self.utils, interval=0.1)
        self.angle_reader.start()
        self.update_angles()
    
    def update_angles(self):
        """更新GUI中的实际角度显示"""
        for motor in self.motors:
            actual_position = motor.get_actual_position()
            actual_angle = self.utils.position_to_angle(actual_position)
            angle_label = self.angle_labels[motor.node_id]
            angle_label.config(text=f"当前角度: {actual_angle:.2f}°")
        self.root.after(100, self.update_angles)  # 每100ms更新一次

def main():
    """主程序，初始化电机并启动GUI"""
    network = canopen.Network()
    network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    utils = MotorUtils()
    try:
        # 初始化4个电机
        motors = []
        for node_id in [0x01, 0x02, 0x03, 0x04]:
            motor = Motor_PP(node_id, network)
            motors.append(motor)
        
        # 创建Tkinter主窗口
        root = tk.Tk()
        app = MotorControlGUI(root, motors, utils)
        root.mainloop()
    except Exception as e:
        logger.error(f"主程序运行出错: {e}")
    finally:
        # 停止角度读取线程
        app.angle_reader.stop()
        # 断开CAN网络连接
        network.disconnect()
        logger.info("已断开与CAN网络的连接")

if __name__ == "__main__":
    main()
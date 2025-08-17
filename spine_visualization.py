import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from collections import deque
import math

class SpineVisualizer:
    def __init__(self, com_port=14, baud_rate=115200, max_points=100):
        # 串口设置
        self.serial_port = serial.Serial(f'COM{com_port}', baud_rate, timeout=0.1)
        self.running = False
        
        # 数据存储
        self.max_points = max_points
        self.time_data = deque(maxlen=max_points)
        self.imu_data = deque(maxlen=max_points)
        
        # 脊柱参数
        self.spine_length = 0.4  # 脊柱总长度 (米)
        self.segment_length = self.spine_length / 4  # 每段长度
        
        # 创建图形
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 设置图形属性
        self.ax.set_xlabel('X (前)')
        self.ax.set_ylabel('Y (左)')
        self.ax.set_zlabel('Z (上)')
        self.ax.set_title('实时脊柱姿态可视化')
        
        # 设置坐标轴范围
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0, 0.5])
        
        # 线程锁
        self.lock = threading.Lock()
        
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """欧拉角转旋转矩阵"""
        # 转换为弧度
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        # 旋转矩阵
        Rx = np.array([[1, 0, 0],
                      [0, math.cos(roll), -math.sin(roll)],
                      [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                      [0, 1, 0],
                      [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                      [math.sin(yaw), math.cos(yaw), 0],
                      [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def calculate_spine_points(self, imu_angles):
        """根据IMU角度计算脊柱各点位置"""
        points = []
        current_pos = np.array([0.0, 0.0, 0.0])
        current_orientation = np.eye(3)
        
        # 计算每个脊柱段的位置
        for i in range(5):
            # 获取当前IMU的欧拉角
            roll, pitch, yaw = imu_angles[i*3:(i+1)*3]
            
            # 计算旋转矩阵
            R = self.euler_to_rotation_matrix(roll, pitch, yaw)
            
            # 更新当前方向
            current_orientation = R @ current_orientation
            
            # 计算下一段的方向向量 (Z轴方向)
            direction = current_orientation @ np.array([0, 0, 1])
            
            # 计算下一段的位置
            next_pos = current_pos + direction * self.segment_length
            
            points.append(current_pos.copy())
            current_pos = next_pos
        
        # 添加最后一个点
        points.append(current_pos)
        
        return np.array(points)
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        # 解析数据
                        data = line.split(',')
                        if len(data) == 16:  # 时间戳 + 5个IMU * 3个角度
                            timestamp = float(data[0])
                            imu_angles = [float(x) for x in data[1:]]
                            
                            with self.lock:
                                self.time_data.append(timestamp)
                                self.imu_data.append(imu_angles)
                                
            except Exception as e:
                print(f"串口读取错误: {e}")
                time.sleep(0.01)
    
    def update_plot(self):
        """更新图形"""
        while self.running:
            try:
                with self.lock:
                    if len(self.imu_data) > 0:
                        # 获取最新的IMU数据
                        latest_angles = self.imu_data[-1]
                        
                        # 计算脊柱点
                        spine_points = self.calculate_spine_points(latest_angles)
                        
                        # 清除之前的图形
                        self.ax.clear()
                        
                        # 设置坐标轴
                        self.ax.set_xlabel('X (前)')
                        self.ax.set_ylabel('Y (左)')
                        self.ax.set_zlabel('Z (上)')
                        self.ax.set_title('实时脊柱姿态可视化')
                        self.ax.set_xlim([-0.3, 0.3])
                        self.ax.set_ylim([-0.3, 0.3])
                        self.ax.set_zlim([0, 0.5])
                        
                        # 绘制脊柱
                        x, y, z = spine_points.T
                        self.ax.plot(x, y, z, 'b-o', linewidth=3, markersize=8, label='脊柱')
                        
                        # 绘制IMU位置
                        for i, point in enumerate(spine_points[:-1]):
                            self.ax.text(point[0], point[1], point[2], f'IMU{i+1}', 
                                       fontsize=8, color='red')
                        
                        # 添加图例
                        self.ax.legend()
                        
                        # 更新显示
                        plt.pause(0.01)
                        
            except Exception as e:
                print(f"图形更新错误: {e}")
                time.sleep(0.01)
    
    def start(self):
        """启动可视化"""
        self.running = True
        
        # 启动数据读取线程
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # 启动图形更新线程
        self.plot_thread = threading.Thread(target=self.update_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()
        
        print("脊柱可视化已启动，按 Ctrl+C 停止...")
        
        try:
            # 主循环
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        """停止可视化"""
        self.running = False
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        plt.close('all')
        print("可视化已停止")

if __name__ == "__main__":
    # 创建可视化器
    visualizer = SpineVisualizer(com_port=14, baud_rate=115200)
    
    # 启动可视化
    visualizer.start()
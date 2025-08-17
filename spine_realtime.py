import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import threading
import time
from collections import deque
import math

class RealtimeSpineVisualizer:
    def __init__(self, com_port=14, baud_rate=115200):
        # 串口设置
        try:
            self.serial_port = serial.Serial(f'COM{com_port}', baud_rate, timeout=0.1)
            print(f"成功连接到COM{com_port}")
        except Exception as e:
            print(f"串口连接失败: {e}")
            return
        
        # 数据存储
        self.latest_angles = None
        self.lock = threading.Lock()
        
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
        
        # 初始化线条对象
        self.spine_line, = self.ax.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='脊柱')
        self.imu_points = []
        for i in range(5):
            point, = self.ax.plot([], [], [], 'ro', markersize=10)
            self.imu_points.append(point)
        
        # 添加文本标签
        self.imu_labels = []
        for i in range(5):
            label = self.ax.text(0, 0, 0, f'IMU{i+1}', fontsize=8, color='red')
            self.imu_labels.append(label)
        
        self.ax.legend()
        
        # 启动数据读取线程
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """欧拉角转旋转矩阵"""
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
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
        if imu_angles is None:
            return np.zeros((6, 3))
        
        points = []
        current_pos = np.array([0.0, 0.0, 0.0])
        current_orientation = np.eye(3)
        
        # 计算每个脊柱段的位置
        for i in range(5):
            roll, pitch, yaw = imu_angles[i*3:(i+1)*3]
            R = self.euler_to_rotation_matrix(roll, pitch, yaw)
            current_orientation = R @ current_orientation
            
            # 计算下一段的方向向量 (Z轴方向)
            direction = current_orientation @ np.array([0, 0, 1])
            next_pos = current_pos + direction * self.segment_length
            
            points.append(current_pos.copy())
            current_pos = next_pos
        
        points.append(current_pos)
        return np.array(points)
    
    def read_serial_data(self):
        """读取串口数据"""
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        data = line.split(',')
                        if len(data) == 16:
                            imu_angles = [float(x) for x in data[1:]]
                            with self.lock:
                                self.latest_angles = imu_angles
                                
            except Exception as e:
                print(f"串口读取错误: {e}")
                time.sleep(0.01)
    
    def update_frame(self, frame):
        """更新动画帧"""
        with self.lock:
            if self.latest_angles is not None:
                spine_points = self.calculate_spine_points(self.latest_angles)
                
                # 更新脊柱线条
                x, y, z = spine_points.T
                self.spine_line.set_data(x, y)
                self.spine_line.set_3d_properties(z)
                
                # 更新IMU点位置
                for i in range(5):
                    self.imu_points[i].set_data([spine_points[i, 0]], [spine_points[i, 1]])
                    self.imu_points[i].set_3d_properties([spine_points[i, 2]])
                    
                    # 更新标签位置
                    self.imu_labels[i].set_position((spine_points[i, 0], spine_points[i, 1]))
                    self.imu_labels[i].set_3d_properties(spine_points[i, 2])
        
        return [self.spine_line] + self.imu_points + self.imu_labels
    
    def start_animation(self):
        """启动动画"""
        print("启动实时可视化，按 Ctrl+C 停止...")
        
        # 创建动画
        ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=50, blit=False
        )
        
        plt.show()
    
    def stop(self):
        """停止可视化"""
        self.running = False
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        plt.close('all')
        print("可视化已停止")

if __name__ == "__main__":
    # 创建可视化器
    visualizer = RealtimeSpineVisualizer(com_port=14, baud_rate=115200)
    
    try:
        # 启动动画
        visualizer.start_animation()
    except KeyboardInterrupt:
        visualizer.stop()
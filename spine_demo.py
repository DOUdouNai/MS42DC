import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import math
import time

class SpineDemo:
    def __init__(self):
        """初始化脊柱演示"""
        # 创建图形
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 设置图形属性
        self.ax.set_xlabel('X (前)')
        self.ax.set_ylabel('Y (左)')
        self.ax.set_zlabel('Z (上)')
        self.ax.set_title('脊柱姿态演示')
        
        # 设置坐标轴范围
        self.ax.set_xlim([-0.3, 0.3])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0, 0.5])
        
        # 脊柱参数
        self.spine_length = 0.4
        self.segment_length = self.spine_length / 4
        
        # 初始化绘图对象
        self.spine_line, = self.ax.plot([], [], [], 'b-o', linewidth=3, markersize=8, label='脊柱')
        
        # 添加IMU标签
        self.imu_labels = []
        for i in range(5):
            label = self.ax.text(0, 0, 0, f'IMU{i+1}', fontsize=8, color='red')
            self.imu_labels.append(label)
        
        self.ax.legend()
        
        # 时间计数器
        self.t = 0
        
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
    
    def generate_demo_data(self):
        """生成演示数据"""
        # 模拟脊柱弯曲运动
        t = self.t * 0.1
        
        # 基础角度
        base_angles = [
            [0, 0, 0],      # IMU1 - 底部
            [5, 2, 0],      # IMU2
            [10, 5, 0],     # IMU3
            [8, 3, 0],      # IMU4
            [3, 1, 0]       # IMU5 - 顶部
        ]
        
        imu_data = []
        for i in range(5):
            base = base_angles[i]
            
            # 添加动态弯曲效果
            bend_factor = math.sin(t + i * 0.5) * 8
            twist_factor = math.cos(t + i * 0.3) * 3
            
            roll = base[0] + bend_factor * (i + 1) * 0.1
            pitch = base[1] + math.sin(t + i) * 3
            yaw = base[2] + twist_factor * (i + 1) * 0.2
            
            imu_data.extend([roll, pitch, yaw])
        
        return imu_data
    
    def calculate_spine_points(self, imu_angles):
        """计算脊柱点位置"""
        points = []
        current_pos = np.array([0.0, 0.0, 0.0])
        current_orientation = np.eye(3)
        
        for i in range(5):
            roll, pitch, yaw = imu_angles[i*3:(i+1)*3]
            R = self.euler_to_rotation_matrix(roll, pitch, yaw)
            current_orientation = R @ current_orientation
            
            direction = current_orientation @ np.array([0, 0, 1])
            next_pos = current_pos + direction * self.segment_length
            
            points.append(current_pos.copy())
            current_pos = next_pos
        
        points.append(current_pos)
        return np.array(points)
    
    def update_frame(self, frame):
        """更新动画帧"""
        # 生成演示数据
        imu_data = self.generate_demo_data()
        
        # 计算脊柱点
        spine_points = self.calculate_spine_points(imu_data)
        
        # 更新脊柱线条
        x, y, z = spine_points.T
        self.spine_line.set_data(x, y)
        self.spine_line.set_3d_properties(z)
        
        # 更新IMU标签
        for i in range(5):
            self.imu_labels[i].set_position((spine_points[i, 0], spine_points[i, 1]))
            self.imu_labels[i].set_3d_properties(spine_points[i, 2])
        
        # 更新时间
        self.t += 1
        
        return [self.spine_line] + self.imu_labels
    
    def start_demo(self):
        """启动演示"""
        print("启动脊柱姿态演示...")
        print("按 Ctrl+C 停止")
        
        # 创建动画
        ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=100, blit=False
        )
        
        plt.show()

def main():
    """主函数"""
    demo = SpineDemo()
    
    try:
        demo.start_demo()
    except KeyboardInterrupt:
        print("演示已停止")

if __name__ == "__main__":
    main()
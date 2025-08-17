import serial
import time
import math
import random

class TestDataGenerator:
    def __init__(self, com_port=14, baud_rate=115200):
        """初始化测试数据生成器"""
        try:
            self.serial_port = serial.Serial(f'COM{com_port}', baud_rate, timeout=0.1)
            print(f"成功连接到COM{com_port}用于发送测试数据")
        except Exception as e:
            print(f"串口连接失败: {e}")
            return
        
        # 初始化角度
        self.base_angles = [
            [0, 0, 0],    # IMU1 - 底部
            [5, 2, 0],    # IMU2
            [10, 5, 0],   # IMU3
            [8, 3, 0],    # IMU4
            [3, 1, 0]     # IMU5 - 顶部
        ]
        
        self.time_counter = 0
        
    def generate_test_data(self):
        """生成测试数据"""
        # 添加一些随机运动
        noise = random.uniform(-2, 2)
        
        # 模拟脊柱弯曲运动
        t = time.time()
        bend_factor = math.sin(t * 0.5) * 5  # 缓慢的弯曲运动
        
        # 生成5个IMU的角度数据
        imu_data = []
        for i in range(5):
            base = self.base_angles[i]
            # 添加弯曲效果和噪声
            roll = base[0] + bend_factor * (i + 1) * 0.2 + noise
            pitch = base[1] + math.sin(t + i) * 2 + noise * 0.5
            yaw = base[2] + math.cos(t + i) * 1 + noise * 0.3
            
            imu_data.extend([roll, pitch, yaw])
        
        return imu_data
    
    def send_data(self):
        """发送数据到串口"""
        while True:
            try:
                # 生成测试数据
                imu_data = self.generate_test_data()
                
                # 构建数据字符串
                timestamp = int(time.time() * 1000)  # 毫秒时间戳
                data_str = f"{timestamp},{','.join([f'{x:.2f}' for x in imu_data])}\n"
                
                # 发送数据
                self.serial_port.write(data_str.encode())
                
                # 打印发送的数据（可选）
                print(f"发送: {data_str.strip()}")
                
                # 控制发送频率
                time.sleep(0.1)  # 100ms间隔
                
            except KeyboardInterrupt:
                print("停止发送测试数据")
                break
            except Exception as e:
                print(f"发送数据错误: {e}")
                break
    
    def close(self):
        """关闭串口连接"""
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
            print("串口连接已关闭")

def main():
    """主函数"""
    print("启动测试数据生成器...")
    print("这将模拟5个IMU的脊柱运动数据")
    print("按 Ctrl+C 停止")
    
    generator = TestDataGenerator(com_port=14, baud_rate=115200)
    
    try:
        generator.send_data()
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        generator.close()

if __name__ == "__main__":
    main()
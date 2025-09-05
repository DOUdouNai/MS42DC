"""
Jaka机器人伺服模式使用示例
解决原始代码卡顿问题的具体应用案例
"""

import time
import math
from jaka_servo_controller import JakaServoController


class JakaServoExamples:
    """Jaka机器人伺服模式使用示例"""
    
    def __init__(self, robot):
        self.robot = robot
        self.servo_controller = JakaServoController(robot)
    
    def example_1_fix_original_stuttering(self):
        """
        示例1：修复原始卡顿代码
        
        原始问题代码：
        robot.servo_p(cartesian_pose=[dx, dy, dz, droll, dpitch, dyaw], move_mode=INCR)
        time.sleep(0.008)  # 这里会导致卡顿
        """
        print("=== 示例1：修复原始卡顿代码 ===")
        
        # 启用伺服模式
        if not self.servo_controller.enable_servo_mode():
            return False
        
        try:
            # 原始问题：直接发送增量指令，没有轨迹规划
            # 修复方案：使用平滑增量运动
            
            # 定义一系列小的增量运动
            movements = [
                (1, 0, 0, 0, 0, 0),    # 沿X轴移动1mm
                (0, 1, 0, 0, 0, 0),    # 沿Y轴移动1mm
                (0, 0, 1, 0, 0, 0),    # 沿Z轴移动1mm
                (0, 0, 0, 0.01, 0, 0), # 绕X轴旋转0.01弧度
                (0, 0, 0, 0, 0.01, 0), # 绕Y轴旋转0.01弧度
                (0, 0, 0, 0, 0, 0.01), # 绕Z轴旋转0.01弧度
            ]
            
            for dx, dy, dz, drx, dry, drz in movements:
                print(f"执行增量运动: dx={dx}, dy={dy}, dz={dz}, drx={drx}, dry={dry}, drz={drz}")
                
                # 使用平滑增量运动替代原始代码
                success = self.servo_controller.smooth_incremental_move(
                    dx, dy, dz, drx, dry, drz, duration=0.5
                )
                
                if not success:
                    print("运动失败")
                    break
                
                time.sleep(0.1)  # 短暂停顿
            
            return True
            
        finally:
            # 禁用伺服模式
            self.servo_controller.disable_servo_mode()
    
    def example_2_circular_motion(self):
        """示例2：圆形轨迹运动"""
        print("=== 示例2：圆形轨迹运动 ===")
        
        with self.servo_controller as servo:
            # 获取当前位置作为圆心
            current_pose = servo.get_current_pose()
            if current_pose is None:
                print("无法获取当前位置")
                return False
            
            center_x, center_y, center_z = current_pose[0], current_pose[1], current_pose[2]
            radius = 20  # 半径20mm
            num_points = 50  # 圆形轨迹点数
            
            print(f"以当前位置为圆心，半径{radius}mm的圆形轨迹")
            
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                z = center_z
                
                target_pose = [x, y, z, current_pose[3], current_pose[4], current_pose[5]]
                
                # 使用平滑运动
                success = servo.smooth_move_to(target_pose, duration=0.1)
                if not success:
                    print(f"圆形轨迹运动失败，点{i}")
                    break
            
            return True
    
    def example_3_sine_wave_motion(self):
        """示例3：正弦波轨迹运动"""
        print("=== 示例3：正弦波轨迹运动 ===")
        
        with self.servo_controller as servo:
            current_pose = servo.get_current_pose()
            if current_pose is None:
                return False
            
            start_x = current_pose[0]
            amplitude = 30  # 振幅30mm
            frequency = 0.5  # 频率0.5Hz
            duration = 4.0  # 总时长4秒
            num_points = int(duration / 0.1)  # 每100ms一个点
            
            print(f"正弦波轨迹：振幅{amplitude}mm，频率{frequency}Hz，时长{duration}秒")
            
            for i in range(num_points):
                t = i * 0.1
                x = start_x + amplitude * math.sin(2 * math.pi * frequency * t)
                y = current_pose[1]
                z = current_pose[2]
                
                target_pose = [x, y, z, current_pose[3], current_pose[4], current_pose[5]]
                
                success = servo.smooth_move_to(target_pose, duration=0.1)
                if not success:
                    print(f"正弦波运动失败，点{i}")
                    break
            
            return True
    
    def example_4_high_frequency_servo(self):
        """示例4：高频伺服运动（解决原始8ms周期问题）"""
        print("=== 示例4：高频伺服运动 ===")
        
        with self.servo_controller as servo:
            # 模拟高频位置更新（如视觉跟踪）
            current_pose = servo.get_current_pose()
            if current_pose is None:
                return False
            
            # 生成随机目标位置序列
            import random
            random.seed(42)  # 固定种子以便复现
            
            num_updates = 100
            max_offset = 5  # 最大偏移5mm
            
            print(f"执行{num_updates}次高频位置更新")
            
            for i in range(num_updates):
                # 生成小的随机偏移
                dx = random.uniform(-max_offset, max_offset)
                dy = random.uniform(-max_offset, max_offset)
                dz = random.uniform(-max_offset, max_offset)
                
                # 使用平滑增量运动
                success = servo.smooth_incremental_move(
                    dx, dy, dz, 0, 0, 0, duration=0.1
                )
                
                if not success:
                    print(f"高频运动失败，更新{i}")
                    break
                
                # 短暂停顿
                time.sleep(0.05)
            
            return True
    
    def example_5_velocity_limited_motion(self):
        """示例5：速度限制运动"""
        print("=== 示例5：速度限制运动 ===")
        
        with self.servo_controller as servo:
            current_pose = servo.get_current_pose()
            if current_pose is None:
                return False
            
            # 尝试大范围运动（可能超过速度限制）
            large_movements = [
                (100, 0, 0, 0, 0, 0),    # 大范围X轴移动
                (0, 100, 0, 0, 0, 0),    # 大范围Y轴移动
                (0, 0, 100, 0, 0, 0),    # 大范围Z轴移动
            ]
            
            for dx, dy, dz, drx, dry, drz in large_movements:
                print(f"尝试大范围运动: dx={dx}, dy={dy}, dz={dz}")
                
                # 使用较长的运动时间以避免速度限制
                success = servo.smooth_incremental_move(
                    dx, dy, dz, drx, dry, drz, duration=3.0
                )
                
                if not success:
                    print("大范围运动失败，可能超过速度限制")
                    break
                
                time.sleep(0.5)
            
            return True
    
    def run_all_examples(self):
        """运行所有示例"""
        examples = [
            self.example_1_fix_original_stuttering,
            self.example_2_circular_motion,
            self.example_3_sine_wave_motion,
            self.example_4_high_frequency_servo,
            self.example_5_velocity_limited_motion,
        ]
        
        for i, example_func in enumerate(examples, 1):
            print(f"\n{'='*50}")
            print(f"运行示例 {i}/{len(examples)}")
            print(f"{'='*50}")
            
            try:
                success = example_func()
                if success:
                    print(f"示例 {i} 执行成功")
                else:
                    print(f"示例 {i} 执行失败")
            except Exception as e:
                print(f"示例 {i} 执行异常: {e}")
            
            # 示例间暂停
            time.sleep(1)


def create_mock_robot():
    """创建模拟机器人对象用于测试"""
    class MockRobot:
        def __init__(self):
            self.current_pose = [300, 200, 400, 0, 0, 0]  # 初始位置
            self.servo_enabled = False
        
        def servo_move_enable(self, enable):
            self.servo_enabled = enable
            print(f"伺服模式: {'启用' if enable else '禁用'}")
            return (0,)  # 成功
        
        def servo_p(self, cartesian_pose, move_mode):
            if not self.servo_enabled:
                return (1, "伺服模式未启用")
            
            if move_mode == 0:  # 绝对运动
                self.current_pose = cartesian_pose
            else:  # 增量运动
                self.current_pose = [c + i for c, i in zip(self.current_pose, cartesian_pose)]
            
            print(f"位置更新: {self.current_pose}")
            return (0,)  # 成功
        
        def get_pose(self):
            return self.current_pose.copy()
    
    return MockRobot()


def main():
    """主函数"""
    print("Jaka机器人伺服模式使用示例")
    print("=" * 50)
    
    # 创建模拟机器人（实际使用时替换为真实的robot实例）
    robot = create_mock_robot()
    
    # 创建示例对象
    examples = JakaServoExamples(robot)
    
    # 运行所有示例
    examples.run_all_examples()
    
    print("\n所有示例执行完成！")


if __name__ == "__main__":
    main()
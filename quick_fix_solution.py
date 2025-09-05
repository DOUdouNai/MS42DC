"""
快速修复方案：直接解决原始卡顿问题
适用于需要立即修复现有代码的场景
"""

import time
import math


def quick_fix_servo_motion(robot, dx, dy, dz, droll, dpitch, dyaw):
    """
    快速修复原始卡顿问题的函数
    
    原始问题代码：
    robot.servo_p(cartesian_pose=[dx, dy, dz, droll, dpitch, dyaw], move_mode=INCR)
    time.sleep(0.008)  # 这里会导致卡顿
    
    修复方案：
    1. 启用伺服模式
    2. 进行轨迹规划
    3. 使用8ms周期发送指令
    4. 禁用伺服模式
    """
    
    # 1. 启用伺服模式
    print("启用伺服模式...")
    result = robot.servo_move_enable(True)
    if result[0] != 0:
        print(f"启用伺服模式失败: {result}")
        return False
    
    try:
        # 2. 获取当前位置
        current_pose = robot.get_pose()  # 根据实际API调整
        if current_pose is None:
            print("无法获取当前位置")
            return False
        
        # 3. 计算目标位置
        target_pose = [
            current_pose[0] + dx,
            current_pose[1] + dy,
            current_pose[2] + dz,
            current_pose[3] + droll,
            current_pose[4] + dpitch,
            current_pose[5] + dyaw
        ]
        
        # 4. 轨迹规划 - 将大运动分解为小运动
        total_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        total_rotation = math.sqrt(droll**2 + dpitch**2 + dyaw**2)
        max_movement = max(total_distance, total_rotation)
        
        # 根据运动幅度确定分段数
        if max_movement > 10:
            num_segments = max(10, int(max_movement / 2))
        else:
            num_segments = max(2, int(max_movement / 0.5))
        
        print(f"运动分段数: {num_segments}")
        
        # 5. 分段执行运动
        for i in range(num_segments):
            # 计算插值比例
            ratio = (i + 1) / num_segments
            
            # 使用平滑插值（S曲线）
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            
            # 计算当前目标位置
            current_target = [
                current_pose[j] + (target_pose[j] - current_pose[j]) * smooth_ratio
                for j in range(6)
            ]
            
            # 发送位置指令
            result = robot.servo_p(
                cartesian_pose=current_target,
                move_mode=0  # 绝对运动
            )
            
            if result[0] != 0:
                print(f"位置指令发送失败: {result}")
                return False
            
            # 8ms控制周期
            time.sleep(0.008)
        
        print("运动完成")
        return True
        
    except Exception as e:
        print(f"运动执行异常: {e}")
        return False
        
    finally:
        # 6. 禁用伺服模式
        print("禁用伺服模式...")
        robot.servo_move_enable(False)


def improved_servo_loop(robot, movement_list):
    """
    改进的伺服运动循环
    
    Args:
        robot: 机器人实例
        movement_list: 运动列表，每个元素为 (dx, dy, dz, droll, dpitch, dyaw)
    """
    
    # 启用伺服模式
    if robot.servo_move_enable(True)[0] != 0:
        print("启用伺服模式失败")
        return False
    
    try:
        for i, (dx, dy, dz, droll, dpitch, dyaw) in enumerate(movement_list):
            print(f"执行运动 {i+1}/{len(movement_list)}: dx={dx}, dy={dy}, dz={dz}")
            
            # 使用快速修复函数
            success = quick_fix_servo_motion(robot, dx, dy, dz, droll, dpitch, dyaw)
            
            if not success:
                print(f"运动 {i+1} 失败")
                break
            
            # 运动间短暂停顿
            time.sleep(0.1)
        
        return True
        
    finally:
        # 禁用伺服模式
        robot.servo_move_enable(False)


# 使用示例
def example_usage():
    """使用示例"""
    
    # 模拟机器人对象
    class MockRobot:
        def __init__(self):
            self.current_pose = [300, 200, 400, 0, 0, 0]
            self.servo_enabled = False
        
        def servo_move_enable(self, enable):
            self.servo_enabled = enable
            return (0,)
        
        def servo_p(self, cartesian_pose, move_mode):
            if move_mode == 0:  # 绝对运动
                self.current_pose = cartesian_pose
            return (0,)
        
        def get_pose(self):
            return self.current_pose.copy()
    
    # 创建模拟机器人
    robot = MockRobot()
    
    # 示例1：单个运动
    print("=== 示例1：单个运动 ===")
    success = quick_fix_servo_motion(robot, 10, 5, 0, 0, 0, 0.1)
    print(f"运动结果: {'成功' if success else '失败'}")
    
    # 示例2：连续运动
    print("\n=== 示例2：连续运动 ===")
    movements = [
        (5, 0, 0, 0, 0, 0),
        (0, 5, 0, 0, 0, 0),
        (0, 0, 5, 0, 0, 0),
        (-5, -5, -5, 0, 0, 0),
    ]
    
    success = improved_servo_loop(robot, movements)
    print(f"连续运动结果: {'成功' if success else '失败'}")


if __name__ == "__main__":
    example_usage()
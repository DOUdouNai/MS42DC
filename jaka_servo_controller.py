"""
Jaka机器人笛卡尔空间伺服模式运动控制器
解决servo_p卡顿问题的完整解决方案

主要功能：
1. 轨迹规划和平滑插补
2. 8ms周期精确控制
3. 关节速度限制检查
4. 自动进入/退出伺服模式
"""

import time
import math
import numpy as np
from typing import List, Tuple, Optional
import threading


class JakaServoController:
    """Jaka机器人伺服模式控制器"""
    
    def __init__(self, robot):
        """
        初始化控制器
        
        Args:
            robot: Jaka机器人实例
        """
        self.robot = robot
        self.is_servo_enabled = False
        self.control_period = 0.008  # 8ms控制周期
        self.max_joint_velocity = 180.0  # 关节速度上限 180度/秒
        self.current_pose = None
        self.target_pose = None
        self.trajectory_thread = None
        self.stop_trajectory = False
        
    def enable_servo_mode(self) -> bool:
        """启用伺服模式"""
        try:
            result = self.robot.servo_move_enable(True)
            if result[0] == 0:
                self.is_servo_enabled = True
                print("伺服模式已启用")
                return True
            else:
                print(f"启用伺服模式失败: {result}")
                return False
        except Exception as e:
            print(f"启用伺服模式异常: {e}")
            return False
    
    def disable_servo_mode(self) -> bool:
        """禁用伺服模式"""
        try:
            result = self.robot.servo_move_enable(False)
            if result[0] == 0:
                self.is_servo_enabled = False
                print("伺服模式已禁用")
                return True
            else:
                print(f"禁用伺服模式失败: {result}")
                return False
        except Exception as e:
            print(f"禁用伺服模式异常: {e}")
            return False
    
    def get_current_pose(self) -> Optional[List[float]]:
        """获取当前笛卡尔位置"""
        try:
            # 假设机器人有获取当前位置的方法
            # 根据实际API调整
            pose = self.robot.get_pose()  # 需要根据实际API调整
            self.current_pose = pose
            return pose
        except Exception as e:
            print(f"获取当前位置失败: {e}")
            return None
    
    def check_joint_velocity_limit(self, start_pose: List[float], 
                                 target_pose: List[float], 
                                 time_duration: float) -> bool:
        """
        检查关节速度是否超过限制
        
        Args:
            start_pose: 起始位置 [x, y, z, rx, ry, rz]
            target_pose: 目标位置 [x, y, z, rx, ry, rz]
            time_duration: 运动时间（秒）
            
        Returns:
            bool: True表示速度在限制内，False表示超限
        """
        try:
            # 计算位置差
            position_diff = [t - s for t, s in zip(target_pose, start_pose)]
            
            # 计算关节角度变化（需要逆运动学）
            # 这里简化处理，实际需要调用逆运动学函数
            joint_diff = self._estimate_joint_angles(position_diff)
            
            # 计算关节速度
            joint_velocities = [abs(diff / time_duration) for diff in joint_diff]
            
            # 检查是否超过限制
            max_velocity = max(joint_velocities)
            if max_velocity > self.max_joint_velocity:
                print(f"警告：关节速度 {max_velocity:.2f} 度/秒 超过限制 {self.max_joint_velocity} 度/秒")
                return False
            
            return True
            
        except Exception as e:
            print(f"速度检查异常: {e}")
            return False
    
    def _estimate_joint_angles(self, cartesian_diff: List[float]) -> List[float]:
        """
        估算关节角度变化（简化版本）
        实际应用中需要调用逆运动学函数
        
        Args:
            cartesian_diff: 笛卡尔空间位置差 [dx, dy, dz, drx, dry, drz]
            
        Returns:
            List[float]: 估算的关节角度变化
        """
        # 这是一个简化的估算，实际需要根据机器人DH参数计算
        # 假设关节角度变化与笛卡尔位置变化成正比
        scale_factor = 0.5  # 需要根据实际机器人调整
        joint_diff = [diff * scale_factor for diff in cartesian_diff]
        
        # 确保有6个关节
        while len(joint_diff) < 6:
            joint_diff.append(0.0)
            
        return joint_diff[:6]
    
    def plan_trajectory(self, start_pose: List[float], 
                       target_pose: List[float], 
                       duration: float) -> List[List[float]]:
        """
        规划平滑轨迹
        
        Args:
            start_pose: 起始位置
            target_pose: 目标位置
            duration: 运动持续时间
            
        Returns:
            List[List[float]]: 轨迹点列表
        """
        # 计算需要的控制点数
        num_points = max(2, int(duration / self.control_period))
        
        # 生成平滑轨迹（使用S曲线）
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            
            # 使用S曲线插值（smoothstep函数）
            smooth_t = t * t * (3.0 - 2.0 * t)
            
            # 线性插值
            interpolated_pose = []
            for j in range(len(start_pose)):
                value = start_pose[j] + (target_pose[j] - start_pose[j]) * smooth_t
                interpolated_pose.append(value)
            
            trajectory.append(interpolated_pose)
        
        return trajectory
    
    def execute_trajectory(self, trajectory: List[List[float]]) -> bool:
        """
        执行轨迹
        
        Args:
            trajectory: 轨迹点列表
            
        Returns:
            bool: 执行是否成功
        """
        if not self.is_servo_enabled:
            print("错误：伺服模式未启用")
            return False
        
        try:
            for i, pose in enumerate(trajectory):
                if self.stop_trajectory:
                    print("轨迹执行被停止")
                    return False
                
                # 发送位置指令
                result = self.robot.servo_p(
                    cartesian_pose=pose,
                    move_mode=0  # 绝对运动
                )
                
                if result[0] != 0:
                    print(f"位置指令发送失败: {result}")
                    return False
                
                # 等待控制周期
                time.sleep(self.control_period)
            
            return True
            
        except Exception as e:
            print(f"轨迹执行异常: {e}")
            return False
    
    def smooth_move_to(self, target_pose: List[float], 
                      duration: float = 1.0) -> bool:
        """
        平滑移动到目标位置
        
        Args:
            target_pose: 目标位置 [x, y, z, rx, ry, rz]
            duration: 运动持续时间（秒）
            
        Returns:
            bool: 运动是否成功
        """
        # 获取当前位置
        current_pose = self.get_current_pose()
        if current_pose is None:
            print("无法获取当前位置")
            return False
        
        # 检查速度限制
        if not self.check_joint_velocity_limit(current_pose, target_pose, duration):
            print("运动速度超过限制，请减小运动幅度或增加运动时间")
            return False
        
        # 规划轨迹
        trajectory = self.plan_trajectory(current_pose, target_pose, duration)
        
        # 执行轨迹
        return self.execute_trajectory(trajectory)
    
    def smooth_incremental_move(self, dx: float, dy: float, dz: float,
                               drx: float, dry: float, drz: float,
                               duration: float = 1.0) -> bool:
        """
        平滑增量运动
        
        Args:
            dx, dy, dz: 位置增量
            drx, dry, drz: 旋转增量
            duration: 运动持续时间（秒）
            
        Returns:
            bool: 运动是否成功
        """
        # 获取当前位置
        current_pose = self.get_current_pose()
        if current_pose is None:
            print("无法获取当前位置")
            return False
        
        # 计算目标位置
        target_pose = [
            current_pose[0] + dx,
            current_pose[1] + dy,
            current_pose[2] + dz,
            current_pose[3] + drx,
            current_pose[4] + dry,
            current_pose[5] + drz
        ]
        
        return self.smooth_move_to(target_pose, duration)
    
    def start_continuous_servo(self, target_pose: List[float], 
                             duration: float = 1.0) -> bool:
        """
        启动连续伺服运动（在后台线程中执行）
        
        Args:
            target_pose: 目标位置
            duration: 运动持续时间
            
        Returns:
            bool: 是否成功启动
        """
        if self.trajectory_thread and self.trajectory_thread.is_alive():
            print("已有轨迹在执行中")
            return False
        
        self.stop_trajectory = False
        self.trajectory_thread = threading.Thread(
            target=self._continuous_servo_worker,
            args=(target_pose, duration)
        )
        self.trajectory_thread.start()
        return True
    
    def _continuous_servo_worker(self, target_pose: List[float], duration: float):
        """连续伺服运动工作线程"""
        try:
            self.smooth_move_to(target_pose, duration)
        except Exception as e:
            print(f"连续伺服运动异常: {e}")
    
    def stop_continuous_servo(self):
        """停止连续伺服运动"""
        self.stop_trajectory = True
        if self.trajectory_thread:
            self.trajectory_thread.join(timeout=1.0)
    
    def __enter__(self):
        """上下文管理器入口"""
        self.enable_servo_mode()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.stop_continuous_servo()
        self.disable_servo_mode()


# 使用示例和测试函数
def example_usage():
    """使用示例"""
    # 假设你已经有了robot实例
    # robot = YourJakaRobot()
    
    # 使用上下文管理器（推荐）
    # with JakaServoController(robot) as servo_controller:
    #     # 平滑移动到目标位置
    #     target_pose = [300, 200, 400, 0, 0, 0]  # [x, y, z, rx, ry, rz]
    #     servo_controller.smooth_move_to(target_pose, duration=2.0)
    #     
    #     # 平滑增量运动
    #     servo_controller.smooth_incremental_move(10, 5, 0, 0, 0, 0.1, duration=1.0)
    
    print("使用示例代码已准备就绪")


if __name__ == "__main__":
    example_usage()
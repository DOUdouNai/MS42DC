#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂控制使用示例
展示如何自定义参数运行机械臂控制
"""

import time
import jkrc
import numpy as np
from robot_spine_control import (
    generate_forward_tilt_trajectory,
    fit_and_sample_trajectory,
    plot_trajectory,
    degrees_to_radians,
    ABS, INCR, ENABLE, DISABLE
)

def custom_spine_movement(robot_ip, tilt_angle=25, arc_radius=60, num_points=80):
    """
    自定义脊柱运动
    
    参数:
    robot_ip (str): 机器人IP地址
    tilt_angle (float): 倾斜角度（度）
    arc_radius (float): 弧线半径（mm）
    num_points (int): 轨迹点数量
    """
    
    try:
        # 连接机器人
        robot = jkrc.RC(robot_ip)
        robot.login()
        robot.power_on()
        robot.enable_robot()
        
        print(f"机器人连接成功！")
        print(f"运动参数: 倾斜角度={tilt_angle}°, 弧线半径={arc_radius}mm, 轨迹点数={num_points}")
        
        # 获取当前位置
        current_pose = robot.get_tcp_position()
        print(f"当前TCP位置: {current_pose}")
        
        # 生成轨迹
        trajectory = generate_forward_tilt_trajectory(
            current_pose,
            tilt_angle=tilt_angle,
            arc_radius=arc_radius,
            num_points=num_points
        )
        
        # 平滑处理
        fitted_trajectory = fit_and_sample_trajectory(
            trajectory, 
            num_points, 
            smoothing_factor=0.05
        )
        
        if fitted_trajectory is None:
            print("轨迹生成失败！")
            return
        
        # 显示轨迹
        plot_trajectory(fitted_trajectory, f"自定义脊柱运动 - 倾斜{tilt_angle}度")
        
        # 执行运动
        robot.servo_move_enable(ENABLE)
        print("开始执行运动...")
        
        prev_pose = fitted_trajectory[0]
        for i in range(len(fitted_trajectory)):
            current_target = fitted_trajectory[i]
            delta_pose = current_target - prev_pose
            
            robot.servo_p(
                cartesian_pose=delta_pose.tolist(),
                move_mode=INCR
            )
            
            prev_pose = current_target
            time.sleep(0.015)  # 稍慢的速度
        
        # 完成运动
        final_pose = robot.get_tcp_position()
        print(f"运动完成！最终位置: {final_pose}")
        
        robot.servo_move_enable(DISABLE)
        
        # 返回起始位置
        print("返回起始位置...")
        robot.linear_move(current_pose, ABS, True, 40)
        time.sleep(2)
        
    except Exception as e:
        print(f"运动过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            robot.logout()
            print("已断开机器人连接")
        except:
            pass

def gentle_spine_movement(robot_ip):
    """温和的脊柱运动（小角度，慢速度）"""
    print("执行温和的脊柱运动...")
    custom_spine_movement(robot_ip, tilt_angle=15, arc_radius=40, num_points=120)

def strong_spine_movement(robot_ip):
    """强烈的脊柱运动（大角度，快速度）"""
    print("执行强烈的脊柱运动...")
    custom_spine_movement(robot_ip, tilt_angle=45, arc_radius=100, num_points=60)

def main():
    """主函数 - 演示不同的运动模式"""
    
    # 机器人IP地址
    robot_ip = "10.5.5.100"  # 请根据实际情况修改
    
    print("=== 机械臂脊柱运动演示 ===\n")
    
    # 选择运动模式
    print("请选择运动模式:")
    print("1. 温和运动 (15度倾斜)")
    print("2. 标准运动 (25度倾斜)")
    print("3. 强烈运动 (45度倾斜)")
    print("4. 自定义参数")
    
    try:
        choice = input("请输入选择 (1-4): ").strip()
        
        if choice == "1":
            gentle_spine_movement(robot_ip)
        elif choice == "2":
            custom_spine_movement(robot_ip, tilt_angle=25, arc_radius=60, num_points=80)
        elif choice == "3":
            strong_spine_movement(robot_ip)
        elif choice == "4":
            # 自定义参数
            tilt_angle = float(input("请输入倾斜角度 (度): "))
            arc_radius = float(input("请输入弧线半径 (mm): "))
            num_points = int(input("请输入轨迹点数: "))
            custom_spine_movement(robot_ip, tilt_angle, arc_radius, num_points)
        else:
            print("无效选择，使用默认参数")
            custom_spine_movement(robot_ip)
            
    except KeyboardInterrupt:
        print("\n用户中断操作")
    except ValueError:
        print("输入参数无效，使用默认参数")
        custom_spine_movement(robot_ip)

if __name__ == "__main__":
    main()
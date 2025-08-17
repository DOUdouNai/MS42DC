#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹生成测试脚本
用于验证轨迹生成功能，不需要连接实际的机器人
"""

import numpy as np
import matplotlib.pyplot as plt
from robot_spine_control import (
    generate_forward_tilt_trajectory,
    fit_and_sample_trajectory,
    plot_trajectory,
    degrees_to_radians
)

def test_trajectory_generation():
    """测试轨迹生成功能"""
    
    # 模拟起始位置 [x, y, z, rx, ry, rz]
    start_pose = [-500, 0, 200, degrees_to_radians(-180), 0, 0]
    
    print("测试参数:")
    print(f"起始位置: {start_pose}")
    print(f"倾斜角度: 30度")
    print(f"弧线半径: 80mm")
    print(f"轨迹点数: 100")
    
    # 生成轨迹
    trajectory = generate_forward_tilt_trajectory(
        start_pose=start_pose,
        tilt_angle=30,
        arc_radius=80,
        num_points=100
    )
    
    print(f"\n生成的轨迹形状: {trajectory.shape}")
    print(f"起始点: {trajectory[0]}")
    print(f"结束点: {trajectory[-1]}")
    
    # 平滑处理
    fitted_trajectory = fit_and_sample_trajectory(
        trajectory, 
        num_points=100, 
        smoothing_factor=0.1
    )
    
    if fitted_trajectory is not None:
        print(f"\n平滑后轨迹形状: {fitted_trajectory.shape}")
        
        # 绘制轨迹
        plot_trajectory(fitted_trajectory, "测试轨迹 - 机械臂向前倾斜30度")
        
        # 分析轨迹变化
        analyze_trajectory(fitted_trajectory)
    else:
        print("轨迹平滑处理失败！")

def analyze_trajectory(trajectory):
    """分析轨迹特征"""
    
    print("\n轨迹分析:")
    
    # 位置变化
    x_start, y_start, z_start = trajectory[0][:3]
    x_end, y_end, z_end = trajectory[-1][:3]
    
    print(f"X轴变化: {x_start:.2f} -> {x_end:.2f} (变化: {x_end-x_start:.2f}mm)")
    print(f"Y轴变化: {y_start:.2f} -> {y_end:.2f} (变化: {y_end-y_start:.2f}mm)")
    print(f"Z轴变化: {z_start:.2f} -> {z_end:.2f} (变化: {z_end-z_start:.2f}mm)")
    
    # 姿态变化
    rx_start, ry_start, rz_start = np.degrees(trajectory[0][3:])
    rx_end, ry_end, rz_end = np.degrees(trajectory[-1][3:])
    
    print(f"RX变化: {rx_start:.2f}° -> {rx_end:.2f}° (变化: {rx_end-rx_start:.2f}°)")
    print(f"RY变化: {ry_start:.2f}° -> {ry_end:.2f}° (变化: {ry_end-ry_start:.2f}°)")
    print(f"RZ变化: {rz_start:.2f}° -> {rz_end:.2f}° (变化: {rz_end-rz_start:.2f}°)")
    
    # 计算总位移
    total_displacement = np.sqrt((x_end-x_start)**2 + (y_end-y_start)**2 + (z_end-z_start)**2)
    print(f"总位移: {total_displacement:.2f}mm")

def test_different_parameters():
    """测试不同参数下的轨迹生成"""
    
    start_pose = [-500, 0, 200, degrees_to_radians(-180), 0, 0]
    
    # 测试不同倾斜角度
    angles = [15, 30, 45, 60]
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()
    
    for i, angle in enumerate(angles):
        trajectory = generate_forward_tilt_trajectory(
            start_pose=start_pose,
            tilt_angle=angle,
            arc_radius=80,
            num_points=100
        )
        
        fitted_trajectory = fit_and_sample_trajectory(trajectory, 100, 0.1)
        
        if fitted_trajectory is not None:
            x = fitted_trajectory[:, 0]
            y = fitted_trajectory[:, 1]
            z = fitted_trajectory[:, 2]
            
            ax = axes[i]
            ax.plot(x, z, 'b-', linewidth=2)
            ax.scatter(x[0], z[0], c='g', s=100, label='起始点')
            ax.scatter(x[-1], z[-1], c='r', s=100, label='结束点')
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Z (mm)')
            ax.set_title(f'倾斜角度: {angle}°')
            ax.legend()
            ax.grid(True)
            ax.set_aspect('equal')
    
    plt.suptitle('不同倾斜角度的轨迹对比')
    plt.tight_layout()
    plt.show()

def main():
    """主函数"""
    print("=== 机械臂轨迹生成测试 ===\n")
    
    # 基本轨迹测试
    print("1. 基本轨迹生成测试")
    test_trajectory_generation()
    
    print("\n" + "="*50 + "\n")
    
    # 不同参数测试
    print("2. 不同参数测试")
    test_different_parameters()
    
    print("\n测试完成！")

if __name__ == "__main__":
    main()
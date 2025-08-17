import time
import jkrc
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 常量定义
PI = 3.1415926
ABS = 0  # 绝对运动模式
INCR = 1  # 增量运动模式
ENABLE = True
DISABLE = False

def degrees_to_radians(degrees):
    """将角度转换为弧度"""
    return degrees / 180.0 * PI

def generate_tilt_trajectory(start_pose, tilt_angle=30, arc_radius=80, num_points=100):
    """
    生成向前倾斜的弧线轨迹
    start_pose: [x, y, z, rx, ry, rz] 起始位置
    tilt_angle: 倾斜角度（度）
    arc_radius: 弧线半径（mm）
    num_points: 轨迹点数量
    """
    # 将倾斜角度转换为弧度
    tilt_rad = degrees_to_radians(tilt_angle)
    
    # 起始位置
    x0, y0, z0, rx0, ry0, rz0 = start_pose
    
    # 生成角度参数
    angles = np.linspace(0, tilt_rad, num_points)
    
    # 计算弧线轨迹（X轴向前，Z轴向下）
    x_trajectory = x0 + arc_radius * np.sin(angles)
    z_trajectory = z0 - arc_radius * (1 - np.cos(angles))
    y_trajectory = np.full(num_points, y0)  # Y坐标保持不变
    
    # 计算姿态变化（绕Y轴旋转实现向前倾斜）
    rx_trajectory = np.full(num_points, rx0)
    ry_trajectory = ry0 + angles  # 绕Y轴旋转
    rz_trajectory = np.full(num_points, rz0)
    
    # 组合轨迹
    trajectory = np.column_stack([
        x_trajectory, y_trajectory, z_trajectory,
        rx_trajectory, ry_trajectory, rz_trajectory
    ])
    
    return trajectory

def plot_trajectory(trajectory):
    """绘制轨迹"""
    fig = plt.figure(figsize=(12, 4))
    
    # 3D轨迹图
    ax1 = fig.add_subplot(131, projection='3d')
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]
    ax1.plot(x, y, z, 'b-', linewidth=2)
    ax1.scatter(x[0], y[0], z[0], c='g', s=100, label='起始点')
    ax1.scatter(x[-1], y[-1], z[-1], c='r', s=100, label='结束点')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('3D轨迹')
    ax1.legend()
    
    # 位置变化图
    ax2 = fig.add_subplot(132)
    steps = np.arange(len(trajectory))
    ax2.plot(steps, x, 'r-', label='X')
    ax2.plot(steps, y, 'g-', label='Y')
    ax2.plot(steps, z, 'b-', label='Z')
    ax2.set_xlabel('步数')
    ax2.set_ylabel('位置 (mm)')
    ax2.set_title('位置变化')
    ax2.legend()
    ax2.grid(True)
    
    # 姿态变化图
    ax3 = fig.add_subplot(133)
    rx = np.degrees(trajectory[:, 3])
    ry = np.degrees(trajectory[:, 4])
    rz = np.degrees(trajectory[:, 5])
    ax3.plot(steps, rx, 'r-', label='RX')
    ax3.plot(steps, ry, 'g-', label='RY')
    ax3.plot(steps, rz, 'b-', label='RZ')
    ax3.set_xlabel('步数')
    ax3.set_ylabel('角度 (度)')
    ax3.set_title('姿态变化')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    try:
        # 机器人连接参数
        robot_ip = "10.5.5.100"  # 修改为你的机器人IP
        
        # 运动参数
        tilt_angle = 30      # 倾斜角度（度）
        arc_radius = 80      # 弧线半径（mm）
        num_points = 100     # 轨迹点数量
        servo_frequency = 0.01  # 伺服运动频率（秒）
        
        # 连接机器人
        print("正在连接机器人...")
        robot = jkrc.RC(robot_ip)
        robot.login()
        robot.power_on()
        robot.enable_robot()
        print("机器人连接成功！")
        
        # 移动到初始位置（可选）
        print("移动到初始位置...")
        initial_position = [-538, -65, 206, degrees_to_radians(-180), 0, 0]
        robot.linear_move(initial_position, ABS, True, 50)
        time.sleep(3)
        
        # 获取当前TCP位置
        current_pose = robot.get_tcp_position()
        print(f"当前TCP位置: {current_pose}")
        
        # 生成向前倾斜轨迹
        print("生成向前倾斜轨迹...")
        trajectory = generate_tilt_trajectory(
            current_pose, 
            tilt_angle=tilt_angle, 
            arc_radius=arc_radius, 
            num_points=num_points
        )
        
        # 绘制轨迹
        plot_trajectory(trajectory)
        
        # 进入伺服运动模式
        robot.servo_move_enable(ENABLE)
        print("已启用伺服运动模式")
        time.sleep(0.5)
        
        # 执行轨迹运动
        print("开始执行向前倾斜运动...")
        prev_pose = trajectory[0]
        
        for i in range(len(trajectory)):
            # 计算增量运动
            current_target = trajectory[i]
            delta_pose = current_target - prev_pose
            
            # 发送伺服运动指令
            robot.servo_p(
                cartesian_pose=delta_pose.tolist(),
                move_mode=INCR
            )
            
            prev_pose = current_target
            time.sleep(servo_frequency)
        
        # 获取移动后的TCP位置
        final_pose = robot.get_tcp_position()
        print(f"最终TCP位置: {final_pose}")
        
        # 退出伺服运动模式
        robot.servo_move_enable(DISABLE)
        print("已禁用伺服运动模式")
        
        # 返回初始位置
        print("返回初始位置...")
        robot.linear_move(current_pose, ABS, True, 50)
        time.sleep(3)
        
        print("运动完成！")
        
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            robot.logout()
            print("已登出机器人")
        except:
            pass

if __name__ == "__main__":
    main()
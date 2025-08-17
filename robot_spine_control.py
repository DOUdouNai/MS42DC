import sys
import time
import jkrc
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from config import *

# 常量定义
PI = 3.1415926
ABS = 0  # 绝对运动模式
INCR = 1  # 增量运动模式
ENABLE = True
DISABLE = False

def degrees_to_radians(degrees):
    """
    将角度转换为弧度。
    
    参数:
    degrees (float): 角度值。
    
    返回:
    float: 弧度值。
    """
    return degrees / 180.0 * PI

def radians_to_degrees(radians):
    """
    将弧度转换为角度。
    
    参数:
    radians (float): 弧度值。
    
    返回:
    float: 角度值。
    """
    return radians * 180.0 / PI

def generate_forward_tilt_trajectory(start_pose, tilt_angle=30, arc_radius=100, num_points=100):
    """
    生成向前倾斜的弧线轨迹，模仿人体头部向前倾斜的运动。
    
    参数:
    start_pose (list): 起始位置 [x, y, z, rx, ry, rz]
    tilt_angle (float): 倾斜角度（度）
    arc_radius (float): 弧线半径（mm）
    num_points (int): 轨迹点数量
    
    返回:
    numpy.ndarray: 轨迹点数组，形状为 (num_points, 6)
    """
    # 将倾斜角度转换为弧度
    tilt_rad = degrees_to_radians(tilt_angle)
    
    # 起始位置
    x0, y0, z0, rx0, ry0, rz0 = start_pose
    
    # 生成角度参数
    angles = np.linspace(0, tilt_rad, num_points)
    
    # 计算弧线轨迹
    # 在XZ平面上生成弧线，X轴向前，Z轴向下
    x_trajectory = x0 + arc_radius * np.sin(angles)
    z_trajectory = z0 - arc_radius * (1 - np.cos(angles))
    y_trajectory = np.full(num_points, y0)  # Y坐标保持不变
    
    # 计算姿态变化（绕Y轴旋转，模拟头部倾斜）
    rx_trajectory = np.full(num_points, rx0)  # 绕X轴旋转保持不变
    ry_trajectory = ry0 + angles  # 绕Y轴旋转，实现向前倾斜
    rz_trajectory = np.full(num_points, rz0)  # 绕Z轴旋转保持不变
    
    # 组合轨迹
    trajectory = np.column_stack([
        x_trajectory, y_trajectory, z_trajectory,
        rx_trajectory, ry_trajectory, rz_trajectory
    ])
    
    return trajectory

def fit_and_sample_trajectory(trajectory, num_points=100, smoothing_factor=0):
    """
    对六维轨迹进行样条插值拟合，并生成指定数量的采样点。
    
    参数:
    trajectory (numpy.ndarray): 原始轨迹坐标，形状为 (N, 6)。
    num_points (int): 采样点的数量，默认为100。
    smoothing_factor (float): 平滑因子。值越大，拟合曲线越平滑。默认为0，表示插值。
    
    返回:
    numpy.ndarray: 拟合并采样后的轨迹坐标，形状为 (num_points, 6)。
    """
    try:
        # 检查输入轨迹的维度
        if trajectory.shape[1] != 6:
            raise ValueError("轨迹数据必须为六维坐标，形状为 (N, 6)。")
        
        # 分离六个维度
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        z = trajectory[:, 2]
        rx = trajectory[:, 3]
        ry = trajectory[:, 4]
        rz = trajectory[:, 5]
        
        # 使用splprep进行参数化拟合
        tck, u = splprep([x, y, z, rx, ry, rz], s=smoothing_factor, per=False)
        
        # 生成新的参数点
        u_new = np.linspace(0, 1, num_points)
        
        # 计算拟合曲线在新的参数点处的坐标
        out = splev(u_new, tck)
        
        # 组合拟合后的坐标
        fitted_trajectory = np.vstack(out).T
        
        return fitted_trajectory
        
    except Exception as e:
        print(f"拟合和采样轨迹时发生错误: {e}")
        return None

def plot_trajectory(trajectory, title="机械臂向前倾斜轨迹"):
    """
    绘制轨迹。
    
    参数:
    trajectory (numpy.ndarray): 轨迹坐标，形状为 (num_points, 6)。
    title (str): 图表标题
    """
    fig = plt.figure(figsize=(15, 5))
    
    # 3D轨迹图
    ax1 = fig.add_subplot(131, projection='3d')
    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]
    ax1.plot(x, y, z, 'b-', linewidth=2, label='轨迹')
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
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

def main():
    try:
        # 创建机器人对象并连接
        robot = jkrc.RC(ROBOT_IP)
        robot.login()  # 登录机器人
        robot.power_on()  # 上电
        robot.enable_robot()  # 使能机器人
        
        print("机器人连接成功！")
        
        # 移动到初始位置（可选）
        if USE_INITIAL_POSITION:
            print("移动到初始位置...")
            robot.linear_move(INITIAL_POSITION, ABS, True, LINEAR_SPEED)
            time.sleep(3)
        
        # 获取当前机械臂的TCP位置
        current_pose = robot.get_tcp_position()
        print(f"当前TCP位置: {current_pose}")
        
        # 生成向前倾斜的轨迹
        print("生成向前倾斜轨迹...")
        
        trajectory = generate_forward_tilt_trajectory(
            current_pose, 
            tilt_angle=TILT_ANGLE, 
            arc_radius=ARC_RADIUS, 
            num_points=NUM_POINTS
        )
        
        # 对轨迹进行平滑处理
        fitted_trajectory = fit_and_sample_trajectory(trajectory, NUM_POINTS, smoothing_factor=SMOOTHING_FACTOR)
        
        if fitted_trajectory is None:
            print("轨迹生成失败！")
            return
        
        # 绘制轨迹
        plot_trajectory(fitted_trajectory, f"机械臂向前倾斜{TILT_ANGLE}度轨迹")
        
        # 进入伺服运动模式
        robot.servo_move_enable(ENABLE)
        print("已启用伺服运动模式")
        time.sleep(0.5)
        
        # 执行轨迹运动
        print("开始执行向前倾斜运动...")
        prev_pose = fitted_trajectory[0]
        
        for i in range(len(fitted_trajectory)):
            # 计算增量运动
            current_target = fitted_trajectory[i]
            delta_pose = current_target - prev_pose
            
            # 发送伺服运动指令
            robot.servo_p(
                cartesian_pose=delta_pose.tolist(),
                move_mode=INCR
            )
            
            prev_pose = current_target
            time.sleep(SERVO_FREQUENCY)  # 控制运动频率
        
        # 获取移动后的TCP位置
        final_pose = robot.get_tcp_position()
        print(f"最终TCP位置: {final_pose}")
        
        # 退出伺服运动模式
        robot.servo_move_enable(DISABLE)
        print("已禁用伺服运动模式")
        
        # 可选：返回初始位置
        if RETURN_TO_START:
            print("返回初始位置...")
            robot.linear_move(current_pose, ABS, True, LINEAR_SPEED)
            time.sleep(3)
        
        print("运动完成！")
        
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            # 登出机器人
            robot.logout()
            print("已登出机器人")
        except:
            pass

if __name__ == "__main__":
    main()
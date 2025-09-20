import sys, time, threading, serial, numpy as np, os, queue, jkrc, math, struct, socket, csv
from scipy.spatial.transform import Rotation as R
from datetime import datetime
from MyPID import (SimplePalpationPID_INCR,low_level_pid,tool_to_world_increment,level_and_depth_final,safe_dpitch,
                   level_and_depth_final_PID,init_spiral_params, LevelingPIDController,level_and_depth_final_PID3)

#描述：放射性轨迹触诊 - 优化版本
# -------------------- 全局变量初始化 --------------------

# -------------------- 常量定义 --------------------
PI = 3.1415926
ABS = 0                  # 机械臂绝对运动模式
INCR = 1                 # 机械臂增量运动模式
ENABLE = True            # 机械臂伺服使能
DISABLE = False          # 机械臂伺服禁用

# 串口传感器参数（优化：降低超时时间）
SERIAL_PORT = 'COM6'     
BAUD_RATE = 115200       
SERIAL_TIMEOUT = 0.01    # 优化：降低超时到10ms

# 控制频率参数
TARGET_CONTROL_FREQ = 100  # 目标控制频率 100Hz
CONTROL_PERIOD = 1.0 / TARGET_CONTROL_FREQ  # 控制周期 10ms

# 高度控制参数
HEIGHT_GAIN = 0.3        
SMOOTH_FACTOR = 0.5      
MOVING_AVG_WINDOW = 3    # 优化：减少窗口大小，提高响应速度
HEIGHT_THRESHOLD = 0.1   

# 六维力传感器参数
FORCE_SENSOR_IP = '192.168.0.108'  
FORCE_SENSOR_PORT = 4008           

# 触头三角形几何参数
SIDE = 20.78             
R_TRIANGLE = SIDE / math.sqrt(3)   
BASE_XY = np.array([
    [R_TRIANGLE, 0.0],
    [-R_TRIANGLE / 2, R_TRIANGLE * math.sqrt(3) / 2],
    [-R_TRIANGLE / 2, -R_TRIANGLE * math.sqrt(3) / 2]
])

def triangle_normal(z1, z2, z3):
    p1 = np.array([BASE_XY[0, 0], BASE_XY[0, 1], z1])
    p2 = np.array([BASE_XY[1, 0], BASE_XY[1, 1], z2])
    p3 = np.array([BASE_XY[2, 0], BASE_XY[2, 1], z3])
    v1, v2 = p2 - p1, p3 - p1
    n = np.cross(v1, v2)
    norm = np.linalg.norm(n)
    return n / norm if norm > 1e-9 else np.array([0., 0., 1.])

def attitude_from_heights(z1, z2, z3):
    nx, ny, nz = triangle_normal(z1, z2, z3)
    roll = math.degrees(math.atan2(ny, nz))
    pitch = math.degrees(math.atan2(-nx, math.hypot(ny, nz)))
    return roll, pitch

def heights_to_roll_pitch_z(d1, d2, d3, threshold=0.1, smooth_factor=0.5, moving_average_window=3):
    """优化版本：减少移动平均窗口，提高响应速度"""
    # 初始化移动平均缓冲区
    if not hasattr(heights_to_roll_pitch_z, "moving_average_buffer"):
        heights_to_roll_pitch_z.moving_average_buffer = []

    roll, pitch = attitude_from_heights(d1, d2, d3)

    # 如果所有传感器读数都小于阈值，返回默认的下降高度
    if abs(d1) < threshold and abs(d2) < threshold and abs(d3) < threshold:
        return roll, pitch, d1, d2, d3, -0.5

    # 计算质心高度
    centroid_z = (d1 + d2 + d3) / 3.0

    # 动态调整高度增益
    dynamic_gain = HEIGHT_GAIN * (1 + abs(centroid_z) * 0.1)

    # 平滑高度变化
    smoothed_z = centroid_z * smooth_factor

    # 移动平均（优化：减少窗口大小）
    heights_to_roll_pitch_z.moving_average_buffer.append(smoothed_z)
    if len(heights_to_roll_pitch_z.moving_average_buffer) > moving_average_window:
        heights_to_roll_pitch_z.moving_average_buffer.pop(0)
    moving_average_z = sum(heights_to_roll_pitch_z.moving_average_buffer) / len(heights_to_roll_pitch_z.moving_average_buffer)

    return roll, pitch, d1, d2, d3, moving_average_z * dynamic_gain

# -------------------- 优化的串口线程 --------------------
def read_sensor_data_optimized(q):
    """
    优化版本：减少串口延迟，提高数据读取效率
    """
    try:
        # 优化：降低超时时间，减少等待延迟
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        print(f"✅ 成功打开触头串口：{SERIAL_PORT}（波特率：{BAUD_RATE}，超时：{SERIAL_TIMEOUT}s）")
    except serial.SerialException as e:
        print(f"❌ 触头串口打开失败：{e}")
        return

    buf = b''
    frame_count = 0
    last_frame_time = time.time()
    
    while True:
        try:
            # 优化：使用read_until，一次性读取到换行符
            line = ser.read_until(b'\n', size=100)  # 限制最大读取长度，避免内存问题
            if line:
                try:
                    # 解码并解析数据
                    data_str = line.decode('utf-8').strip()
                    parts = data_str.split(',')
                    if len(parts) >= 4:
                        d1, d2, d3 = map(float, parts[1:4])
                        
                        # 处理高度数据
                        roll, pitch, d1, d2, d3, delta_z = heights_to_roll_pitch_z(d1, d2, d3)
                        
                        # 优化：使用非阻塞put，避免队列阻塞
                        try:
                            q.put_nowait((roll, pitch, d1, d2, d3, delta_z, time.time()))
                        except queue.Full:
                            # 队列满时，丢弃最旧的数据，保持最新数据
                            try:
                                q.get_nowait()
                                q.put_nowait((roll, pitch, d1, d2, d3, delta_z, time.time()))
                            except queue.Empty:
                                pass
                        
                        # 统计帧率（每100帧打印一次）
                        frame_count += 1
                        if frame_count % 100 == 0:
                            current_time = time.time()
                            fps = 100 / (current_time - last_frame_time)
                            print(f"📊 串口数据帧率：{fps:.1f} Hz")
                            last_frame_time = current_time
                            
                except Exception as e:
                    print(f"⚠️  数据解析失败：{e}")
                    
        except Exception as e:
            print(f"⚠️  串口读取异常：{e}")
            time.sleep(0.001)

# -------------------- 优化的机械臂控制线程 --------------------
def control_robot_optimized(q, q_force, robot):
    """
    优化版本：固定频率控制，减少延迟累积
    """
    # 初始化变量
    prev_roll, prev_pitch = 0.0, 0.0
    last_control_time = time.time()
    control_count = 0
    
    # 优化：使用固定时间间隔控制
    next_control_time = time.time()
    
    while True:
        current_time = time.time()
        
        # 等待到下一个控制周期
        if current_time < next_control_time:
            time.sleep(max(0, next_control_time - current_time))
        
        next_control_time += CONTROL_PERIOD
        
        # 获取最新的传感器数据
        latest_data = None
        data_count = 0
        
        # 优化：清空队列，只使用最新数据
        try:
            while True:
                latest_data = q.get_nowait()
                data_count += 1
        except queue.Empty:
            pass
        
        if latest_data is None:
            # 无数据时使用默认值
            roll, pitch, d1, d2, d3, delta_z, timestamp = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, time.time()
        else:
            roll, pitch, d1, d2, d3, delta_z, timestamp = latest_data
        
        # 计算数据延迟
        data_age = time.time() - timestamp if latest_data else 0
        
        # 优化：使用增量控制，减少计算量
        droll = roll - prev_roll
        dpitch = pitch - prev_pitch
        prev_roll, prev_pitch = roll, pitch
        
        # 优化：限制角度变化幅度，避免剧烈运动
        max_angle_change = 2.0  # 最大角度变化 2度
        droll = max(-max_angle_change, min(max_angle_change, droll))
        dpitch = max(-max_angle_change, min(max_angle_change, dpitch))
        
        # 构建目标姿态
        cartesian_pose = [
            -462, -20, 160 + 50,
            math.radians(180 + roll), 
            math.radians(-pitch), 
            math.radians(179)
        ]
        
        # 优化：使用更高效的伺服控制
        try:
            robot.servo_p_extend(cartesian_pose, ABS, 1)
        except Exception as e:
            print(f"⚠️  机械臂控制异常：{e}")
        
        # 统计控制频率（每100次打印一次）
        control_count += 1
        if control_count % 100 == 0:
            actual_freq = 100 / (time.time() - last_control_time)
            print(f"🎮 控制频率：{actual_freq:.1f} Hz | 数据延迟：{data_age*1000:.1f}ms | 丢弃数据：{data_count-1}")
            last_control_time = time.time()

# -------------------- 主程序 --------------------
def main():
    robot = jkrc.RC("10.5.5.100")
    robot.login()
    robot.power_on()
    robot.enable_robot()

    # 设置工具坐标系
    probe_tcp = [0, 0, 180, 0, 0, 0]
    robot.set_tool_data(1, probe_tcp, "touch_probe")
    robot.set_tool_id(1)
    
    ret = robot.get_tool_id()
    if ret[0] == 0:
        current_tool_id = ret[1]
        print("当前激活的工具ID:", current_tool_id)
    else:
        print("获取工具ID失败，错误码:", ret[0])
    
    time.sleep(0.5)
    
    # 安全运动
    current = robot.get_tcp_position()
    if current[0] == 0:
        current_pose = current[1]
        initial_pose0 = [current_pose[0], current_pose[1], current_pose[2] + 50, 
                        math.radians(180), math.radians(0), math.radians(180)]
        print("安全运动，向上避障")
        robot.linear_move(initial_pose0, ABS, True, 10)
        time.sleep(3)
        print("到达初始位置")
    else:
        print("Failed to get current TCP position")
    
    # 到达触诊位置
    initial_pose0 = [-462, -20, 160+50, math.radians(179.5), math.radians(0), math.radians(179)]
    robot.linear_move(initial_pose0, ABS, True, 10)
    time.sleep(3)
    print("到达手动示教触诊位置")

    # 启动伺服控制
    robot.servo_move_enable(ENABLE)
    
    # 优化：使用固定大小的队列，避免内存积压
    q = queue.Queue(maxsize=10)  # 限制队列大小
    q_force = queue.Queue(maxsize=10)
    
    # 启动优化的线程
    threading.Thread(target=read_sensor_data_optimized, args=(q,), daemon=True).start()
    threading.Thread(target=control_robot_optimized, args=(q, q_force, robot), daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.servo_move_enable(DISABLE)
        robot.logout()

if __name__ == "__main__":
    main()
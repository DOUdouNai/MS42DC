# 机械臂控制配置文件

# 机器人连接配置
ROBOT_IP = "10.5.5.100"  # 机器人IP地址

# 运动参数
TILT_ANGLE = 30          # 倾斜角度（度）
ARC_RADIUS = 80          # 弧线半径（mm）
NUM_POINTS = 100         # 轨迹点数量
SMOOTHING_FACTOR = 0.1   # 轨迹平滑因子

# 运动速度参数
LINEAR_SPEED = 50        # 直线运动速度（mm/s）
SERVO_FREQUENCY = 0.01   # 伺服运动频率（秒）

# 初始位置配置（可选）
USE_INITIAL_POSITION = True
INITIAL_POSITION = [-538, -65, 206, -3.14159, 0, 0]  # [x, y, z, rx, ry, rz]

# 运动模式
RETURN_TO_START = True   # 运动完成后是否返回起始位置
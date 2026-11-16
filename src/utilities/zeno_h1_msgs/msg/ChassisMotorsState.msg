# 底盘所有电机状态消息
std_msgs/Header header

# 电机是否使能
bool motors_enabled
 # 急停状态
bool emergency_stop

# 轮毂电机状态（4个）
MotorState[] wheel_motors
# 转向电机状态（4个）
MotorState[] steering_motors
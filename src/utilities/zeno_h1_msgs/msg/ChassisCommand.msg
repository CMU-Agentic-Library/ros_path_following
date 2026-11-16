# 底盘控制指令消息（订阅）
# 包含所有控制接口：运动控制、使能/失能、急停
std_msgs/Header header

# 运动控制
float64 linear_x           # X方向线速度 (m/s)
float64 linear_y           # Y方向线速度 (m/s)
float64 angular_z          # 角速度 (rad/s)

# 电机使能控制
bool enable_motors         # true=使能, false=失能
# 急停控制
bool emergency_stop        # true=急停, false=正常
# 电机命令消息
# 电机标识（例如："left_arm_j0", "right_gripper"）
string motor_id

# 控制模式选择
# 0 - 空闲模式
# 2 - 速度控制
# 3 - 位置控制
# 4 - MIT / 运控模式（达妙 MIT，高擎运控）
# 5 - 力位混控 / 协同模式
# 255 - 未知模式
uint8 control_mode

# 控制值（解释取决于control_mode）
float64 position         # 目标位置（弧度）（用于MODE_POSITION, MODE_MIT, MODE_FORCE_POSITION）
float64 velocity         # 目标/限制速度（rad/s）（用于MODE_VELOCITY, MODE_POSITION, MODE_MIT, MODE_FORCE_POSITION）
float64 torque           # 目标/前馈力矩（Nm）（用于MODE_TORQUE, MODE_MIT, MODE_FORCE_POSITION）

# MIT控制增益（用于MODE_MIT）
float64 kp               # 位置增益
float64 kd               # 速度增益

# 限制（可选，用于MODE_FORCE_POSITION）
float64 velocity_limit   # 速度限制（rad/s）
float64 torque_limit     # 力矩限制（Nm）
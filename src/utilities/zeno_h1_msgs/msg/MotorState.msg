# 电机状态消息
std_msgs/Header header

# 电机标识
string motor_id          # 电机标识符
string name                # 电机名称
uint8 id                   # 电机ID
uint8 can_id             # CAN总线ID
uint8 motor_type           # 电机类型: 0=轮毂电机, 1=转向电机

# 位置和速度
float64 position         # 当前位置（弧度）
float64 velocity         # 当前速度（rad/s）
float64 torque           # 当前力矩（Nm）
float64 current          # 当前电流（安培）

# 温度
int8 temp_mos            # MOS温度（摄氏度）
int8 temp_motor          # 电机温度（摄氏度）

# 状态
bool is_enabled          # 电机是否使能
bool is_connected        # 电机是否连接（false表示断联/超时）
uint8 error_code         # 错误码（0 = 无错误）
string error_msg         # 错误信息描述

# 当前工作模式
# 0 - 空闲模式
# 2 - 速度控制
# 3 - 位置控制
# 4 - MIT / 运控模式（达妙 MIT，高擎运控）
# 5 - 力位混控 / 协同模式
# 255 - 未知模式
uint8 work_mode
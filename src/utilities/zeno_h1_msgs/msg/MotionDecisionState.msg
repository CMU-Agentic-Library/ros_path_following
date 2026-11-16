# 运控决策状态
std_msgs/Header header

# 控制模式
# 0 - 空闲
# 1 - 自主运控
# 2 - APP控制
# 3 - 全身遥操
# 4 - VR遥操
# 5 - 手柄控制
uint8 control_mode

# VR子模式
# 0 - 默认复位
# 1 - 双臂锁定
# 2 - 正常解算
# 3 - 仅左臂【右臂锁定】
# 4 - 仅右臂【左臂锁定】
# 5 - 标定复位【肘关节90°】
uint8 vr_sub_mode
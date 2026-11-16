# 电量百分比 SOC (%)
float32 soc_percentage
# 充放电状态 0=静止 1=放电 2=充电
uint8 charge_discharge_status
# 温度 (℃)
float32 temperature
# 电压 (V)
float32 voltage
# 报警状态 bit0=过压 bit1=过流 bit2=高温 bit3=低压 bit4=低电量
uint8 alarm_status
# 充放电剩余时间 (秒)
float32 remaining_time
# 功率 (W)
float32 power

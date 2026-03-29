uint64 timestamp          # 系统自启动以来的时间 (微秒)

uint32 device_id          # 唯一设备 ID ,用于区分同类设备的多个实例

# 编码器类型枚举
uint8 TYPE_ABSOLUTE = 0   # 绝对式编码器
uint8 TYPE_INCREMENTAL = 1# 增量式/相对式编码器
uint8 type                # 当前记录的编码器类型

# 核心物理量 (严格遵守 SI 国际单位制)
float32 position_rad      # 当前绝对角度或累计相对角度 (弧度 Rad)
float32 velocity_rad_s    # 当前角速度 (弧度/秒 Rad/s)，若硬件无法直接测速可填 NaN

# 硬件相关特征 (保留这些字段有助于上层做滤波或诊断)
int32 raw_value           # 传感器寄存器原始读数 (12位绝对式编码器为 0-4095，或增量编码器的脉冲计数值)
uint32 resolution         # 编码器单圈分辨率 (12位绝对式编码器 为 4096，增量式为 PPR)

# 健康与状态指示
uint8 error_count         # I2C/SPI 等底层通信连续错误计数
uint8 status_flags        # 硬件特定的状态标志位 (0 表示正常)

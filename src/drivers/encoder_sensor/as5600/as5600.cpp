#include "as5600.hpp"

AS5600::AS5600(const I2CSPIDriverConfig &config) :
    device::I2C(config),
    I2CSPIDriver<AS5600>(config)
{

}

int AS5600::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
        return ret; // 如果探不到芯片，直接退出
    }

    // 2. 核心调度指令：要求操作系统每隔 10000 微秒 (即 100Hz) 自动调用一次 RunImpl()
    ScheduleOnInterval(10000);

    return PX4_OK;
}

// 探测函数：开机时检查设备是否在线
int AS5600::probe()
{
    uint8_t cmd = AS5600_REG_STATUS;
    uint8_t val = 0;
    // 尝试读取一次状态寄存器，如果成功返回 OK (0)
    int ret = transfer(&cmd, 1, &val, 1);
    return ret;
}

void AS5600::RunImpl()
{
    // 1. 读取 STATUS 寄存器 (磁铁健康状态)
    uint8_t reg_status_cmd = AS5600_REG_STATUS;
    uint8_t status_val = 0;
    if (transfer(&reg_status_cmd, 1, &status_val, 1) != PX4_OK) {
        _comms_errors++;
    }

    // 2. 读取 12 位角度寄存器
    uint8_t reg_angle_cmd = AS5600_REG_ANGLE_H;
    uint8_t angle_data[2] = {0, 0};

    // AS5600 支持地址自增，可以直接连续读取 0x0E 和 0x0F
    if (transfer(&reg_angle_cmd, 1, angle_data, 2) != PX4_OK) {
        _comms_errors++;
        return; // 读取失败则跳过本次发布
    }

    // --- 数据解析与计算 ---
    uint16_t raw_angle = ((uint16_t)(angle_data[0] & 0x0F) << 8) | angle_data[1];
    float current_angle_rad = (float)raw_angle * (2.0f * M_PI_F / 4096.0f);
    uint64_t now = hrt_absolute_time();

    // 测速解算与过零点处理 (防止 359度跳变到1度 时产生极大的错误速度)
    float velocity_rad_s = NAN; // 默认填入无效值 (Not a Number)
    if (_last_timestamp != 0) {
        float dt = (now - _last_timestamp) / 1e6f; // 转换为秒
        if (dt > 0.0f) {
            float delta_angle = current_angle_rad - _last_angle_rad;

            // 关键逻辑：处理 0-2PI 的边界跳变
            if (delta_angle < -M_PI_F) { delta_angle += 2.0f * M_PI_F; }
            else if (delta_angle > M_PI_F) { delta_angle -= 2.0f * M_PI_F; }

            velocity_rad_s = delta_angle / dt;
        }
    }

    // 更新状态缓存
    _last_timestamp = now;
    _last_angle_rad = current_angle_rad;

    // --- 组装通用的 uORB 消息 ---
    sensor_encoder_s msg{};
    msg.timestamp = now;

    // 获取由系统自动分配的、具有总线物理特征的唯一 ID
    msg.device_id = get_device_id();

    msg.type = sensor_encoder_s::TYPE_ABSOLUTE;
    msg.position_rad = current_angle_rad;
    msg.velocity_rad_s = velocity_rad_s;
    msg.raw_value = raw_angle;
    msg.resolution = 4096;
    msg.error_count = _comms_errors;

    // 解析 AS5600 特有的状态标志 (Bit 5: 磁铁过强, Bit 4: 磁铁过弱, Bit 3: 未检测到磁铁)
    // 我们将这些报警位直接映射到 status_flags
    msg.status_flags = (status_val & 0x38); // 如果为 0，代表磁铁距离完美

    // 发布消息
    _encoder_pub.publish(msg);

    debug_value_s dbg_msg{};
    dbg_msg.timestamp = hrt_absolute_time();
    dbg_msg.value = current_angle_rad;
    dbg_msg.ind = 1;
    _debug_pub.publish(dbg_msg);
}

I2CSPIDriverBase *AS5600::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
    // 拷贝一份配置，并强制塞入我们定义好的 AS5600 I2C 硬件地址 (0x36)
    I2CSPIDriverConfig my_config = config;
    my_config.i2c_address = AS5600_I2C_ADDR;

    AS5600 *instance = new AS5600(my_config);
    if (!instance) {
        PX4_ERR("alloc failed");
        return nullptr;
    }
    if (instance->init() != PX4_OK) {
        delete instance;
        return nullptr;
    }
    return instance;
}

void AS5600::print_status()
{
    // 调用系统基类的标准状态打印,自动打印出运行频率、I2C 端口号等信息
    I2CSPIDriverBase::print_status();

    // 打印专属信息
    PX4_INFO("AS5600 Magnetic Encoder is running smoothly.");
    PX4_INFO("Current Position (rad): %.3f", (double)_last_angle_rad);
}

void AS5600::print_usage()
{
    PRINT_MODULE_USAGE_NAME("as5600", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("sensor");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int as5600_main(int argc, char *argv[])
{
    using ThisDriver = AS5600;

    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 400000;

    // 请确保 AS5600_I2C_ADDR 已经在你的代码中被定义 (例如 0x36)
    cli.i2c_address = AS5600_I2C_ADDR;

    // 解析出 "-X", "-q" 等标志位，并提取出核心动作指令 (start/stop/status)
    const char *verb = cli.parseDefaultArguments(argc, argv);
    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    // --- 核心修正点：实例化总线迭代器 ---
    // 第三个参数用于指定设备类型 ID (DRV_..._DEVTYPE)。
    // 由于 AS5600 是我们自定义的外设，系统枚举中大概率没有专属它的宏定义，
    // 所以这里直接传入 0 (相当于 DRV_ANY_DEVTYPE)，让它在所选总线上进行泛型探测。
    BusInstanceIterator iterator(MODULE_NAME, cli, 0);

    // 将指令与迭代器同时下发给相应的模板处理函数
    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    // 未知指令
    ThisDriver::print_usage();
    return -1;
}

#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_encoder.h> // 引入你新定义的通用消息
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/debug_value.h>

// AS5600 寄存器地址定义
#define AS5600_I2C_ADDR      0x36
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_ANGLE_H   0x0E
#define AS5600_REG_ANGLE_L   0x0F

class AS5600 : public device::I2C, public I2CSPIDriver<AS5600>
{
public:
    AS5600(const I2CSPIDriverConfig &config);
    ~AS5600() override = default;

    static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
    static void print_usage();

    int init();

    void RunImpl();

    void print_status() override;

private:
    int probe();

    // uORB 发布器 (支持多实例)
    uORB::PublicationMulti<sensor_encoder_s> _encoder_pub{ORB_ID(sensor_encoder)};
    uORB::Publication<debug_value_s> _debug_pub{ORB_ID(debug_value)};

    // 用于计算角速度的状态变量
    uint64_t _last_timestamp{0};
    float _last_angle_rad{0.0f};

    // I2C 错误计数
    uint8_t _comms_errors{0};
};

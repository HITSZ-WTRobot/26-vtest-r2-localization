/**
 * @file    protocol.cpp
 * @author  syhanjin
 * @date    2026-02-24
 */
#include "protocol.hpp"

#include "chassis.hpp"
#include "device.hpp"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "LocEKF.hpp"
#include "system.hpp"

#include <cstring>

namespace Protocol
{

namespace
{
uint32_t read_u32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) << 24 | data[1] << 16 | data[2] << 8 | data[3];
}
uint16_t read_u16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) << 8 | data[1];
}
int16_t read_i16(const uint8_t* data)
{
    return static_cast<int16_t>(data[0] << 8 | data[1]);
}
} // namespace

bool PCProtocol::decode(const uint8_t data[19])
{
    // check CRC
    const uint16_t crc_in_data = read_u16(&data[17]);

    const uint16_t crc = CRC16_Modbus::calc(data, 17);
    if (crc != crc_in_data)
    {
        return false;
    }
    rx_buffer_.push(
            [&](Frame& f)
            {
                f.rx_timestamp = HAL_GetTick();
                f.cmd          = data[0];
                f.tx_timestamp = read_u32(&data[13]);
                f.crc16        = crc_in_data;
                memcpy(f.data, data + 1, 2 * 6);
            });
    return true;
}

void PCProtocol::cmdHandler(Frame& f)
{
    const auto [rx_timestamp, cmd, data, tx_timestamp, crc] = f;

    clock_.align(static_cast<float>(rx_timestamp),
                 static_cast<float>(tx_timestamp) + pc_rx->transitionDelayMS());

    msg_cnt_++;
    if (msg_cnt_ < 50)
    {
        // 还在对时阶段，丢弃消息
        return;
    }

    constexpr auto toPos   = [](const int16_t a) { return static_cast<float>(a) / 2000.0f; };
    constexpr auto toAngle = [](const int16_t a) { return static_cast<float>(a) / 100.0f; };
    constexpr auto toVel   = [](const int16_t a) { return static_cast<float>(a) / 2000.0f; };

    switch (cmd)
    {
    case 0x01: // ping
        break;
    case 0x10: // 停止底盘
        Chassis::ctrl->stop();
        break;
    case 0x11: // 重设坐标系
    {
        Chassis::ctrl->stop();
        // sensor_ops->resetWorldCoordByPose({ value[0], value[1], value[2] });
        break;
    }
    case 0x12: // push 底盘轨迹点
    {
        // const auto value = as_float6(frame.data);
        // chassis_->pushTrajectoryPoint(
        //         { value[0], value[1], value[2], value[3], value[4], value[5] });
        // break;
    }
    case 0x21: // 雷达定位点
    {
        const chassis::Posture pos = { toPos(read_i16(&data[0])),
                                       toPos(read_i16(&data[2])),
                                       toAngle(read_i16(&data[4])) };

        debug_.lidar.last_rcv_posture = pos;

        const uint32_t lidar_time = read_u32(data + 6);

        const auto lidar_self_time = clock_.pcTime2SelfTime(lidar_time);

        debug_.lidar.last_rcv_posture_timestamp = lidar_self_time;
        debug_.lidar.last_rcv_timestamp         = HAL_GetTick();
        debug_.lidar.last_rcv_delay = static_cast<int>(debug_.lidar.last_rcv_timestamp) -
                                      static_cast<int>(lidar_self_time);

        if (!System::Init::posReceived)
        {
            // 进入初始化流程
            if (!Device::sensor::gyro_yaw || !Device::sensor::gyro_yaw->isConnected())
                // 必须确保陀螺仪先连接才可初始化
                return;

            System::Init::pos         = pos;
            System::Init::posReceived = true;

            System::Init::initPosReceived();
        }
        else
        {
            if (Chassis::loc != nullptr)
                Chassis::loc->updateLidar(pos, lidar_self_time);
        }
    }
    break;
    default:;
    }
}

[[noreturn]]
void PCProtocol::loop()
{
    for (;;)
    {
        while (!pc_rx->rx_buffer_.empty())
        {
            const auto f = pc_rx->rx_buffer_.pop();
            cmdHandler(*f);
        }
        osDelay(1);
    }
}

osThreadAttr_t processor_attr{
    .name       = "pc-cmd-processor",
    .stack_size = 1024 * 4,
    .priority   = osPriorityRealtime,
};

void init()
{
    UartRxSync_RegisterCallback(pc_rx, config::uart::PCUart);

    pc_rx = new PCProtocol(config::uart::PCUart);

    if (!pc_rx->startReceive())
        Error_Handler();

    osThreadNew(PCProtocol::TaskEntry, pc_rx, &processor_attr);
}

} // namespace Protocol
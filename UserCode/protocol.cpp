/**
 * @file    protocol.cpp
 * @author  syhanjin
 * @date    2026-02-24
 */
#include "protocol.hpp"

#include "chassis.hpp"
#include "device.hpp"
#include "usart.h"
#include "LocEKF.hpp"

#include <cstring>

PCProtocol* pc_rx;
Clock*      clock_;
UartRxSync_DefineCallback(pc_rx);

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

[[noreturn]]
void PC_CMD_Processor(void* argument)
{
    for (;;)
    {
        while (!pc_rx->rx_buffer_.empty())
        {
            const auto f = pc_rx->rx_buffer_.pop();

            const auto [rx_timestamp, cmd, data, tx_timestamp, crc] = *f;

            clock_->align(static_cast<float>(rx_timestamp),
                          static_cast<float>(tx_timestamp) + pc_rx->transitionDelayMS());

            constexpr auto toPos = [](const uint16_t a) { return static_cast<float>(a) / 2000.0f; };
            constexpr auto toVel = [](const uint16_t a) { return static_cast<float>(a) / 2000.0f; };

            switch (cmd)
            {
            case 0x01: // ping
                break;
            case 0x10: // 停止底盘
                chassis_->stop();
                break;
            case 0x11: // 重设坐标系
            {
                chassis_->stop();
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
                const chassis::Posture pos = { toPos(read_u16(&data[0])),
                                               toPos(read_u16(&data[2])),
                                               toPos(read_u16(&data[4])) };

                const uint32_t lidar_time = read_u32(data + 6);

                const auto lidar_self_time = clock_->pcTime2SelfTime(lidar_time);

                loc_ekf->updateLidar(pos, lidar_self_time);
                break;
            }
            default:;
            }
        }
        osDelay(1);
    }
}

osThreadAttr_t processor_attr{
    .name       = "pc-cmd-processor",
    .stack_size = 256 * 4,
    .priority   = osPriorityRealtime,
};

void Protocol_Init()
{
    UartRxSync_RegisterCallback(pc_rx, PCUart);

    pc_rx = new PCProtocol(PCUart);

    clock_ = new Clock();

    if (!pc_rx->startReceive())
        Error_Handler();

    osThreadNew(PC_CMD_Processor, nullptr, &processor_attr);
}
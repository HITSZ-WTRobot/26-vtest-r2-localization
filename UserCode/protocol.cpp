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

PCProtocol* pc_rx;
UartRxSync_DefineCallback(pc_rx);

bool PCProtocol::decode(const uint8_t data[27])
{
    // check CRC
    if (CRC16_Modbus::calc(data, 25) != *reinterpret_cast<const uint16_t*>(data + 25))
    {
        return false;
    }
    rx_buffer_.push(*reinterpret_cast<const Frame*>(data));
    return true;
}

static std::array<float, 6> as_float6(const uint8_t* data)
{
    std::array<float, 6> result{};
    for (int i = 0; i < 6; ++i)
        result[i] = *reinterpret_cast<const float*>(data + i * sizeof(float));
    return result;
}

void PC_CMD_Processor(void* argument)
{
    for (;;)
    {
        while (!pc_rx->rx_buffer_.empty())
        {
            PCProtocol::Frame frame{};
            pc_rx->rx_buffer_.pop(frame);

            switch (frame.cmd)
            {
            case 0x10: // 停止底盘
                chassis_->stop();
                break;
            case 0x11: // 重设坐标系
            {
                const auto value = as_float6(frame.data);
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
                const auto                         value      = as_float6(frame.data);
                const float                        timestamp  = value[0];
                const float                        lidar_time = value[1];
                const chassis_loc::LocEKF::Posture pos        = { value[2], value[3], value[4] };
                // TODO: 时间对齐
                loc_ekf->updateLidar(pos, static_cast<uint32_t>(lidar_time * 1000));
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

    if (!pc_rx->startReceive())
        Error_Handler();

    osThreadNew(PC_CMD_Processor, nullptr, &processor_attr);
}
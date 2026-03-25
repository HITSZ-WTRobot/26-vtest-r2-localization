/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#include "device.hpp"
#include "can.h"
#include "cmsis_os2.h"

namespace Device
{

namespace
{

void sensor_init()
{
    using sensors::gyro::HWT101CT;

    sensor::gyro_yaw = new HWT101CT(config::uart::SensorGyroYawUart);
    UartRxSync_RegisterCallback(Device::sensor::gyro_yaw, config::uart::SensorGyroYawUart);

    if (!sensor::gyro_yaw->startReceive())
        Error_Handler();
}

void can_init()
{
    // CAN 初始化
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    // motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    // CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    // 注册 CAN 主回调，并启动 CAN
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
    // CAN_Fifo0ReceiveCallback); CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_wheel_config[4] = {
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 1,
            .reverse = false,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 2,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 3,
            .reverse = true,
    },
    {
            .hcan    = &hcan1,
            .type    = motors::DJIMotor::Type::M3508_C620,
            .id1     = 4,
            .reverse = false,
    },
};

void motor_wheel_init()
{
    using motors::DJIMotor;
    for (size_t i = 0; i < 4; ++i)
        motor::wheel[i] = new DJIMotor(motor_wheel_config[i]);
}
} // namespace

void init()
{
    sensor_init();

    can_init();

    motor_wheel_init();
}

void update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool isAllConnected()
{
    constexpr auto def_and_connected = [](auto a) { return a && a->isConnected(); };

    // sensor
    if (!def_and_connected(Device::sensor::gyro_yaw))
        return false;

    for (auto& m : Device::motor::wheel)
        if (!def_and_connected(m))
            return false;

    return true;
}
void waitAllConnected()
{
    while (!isAllConnected())
        osDelay(1);
}

} // namespace Device
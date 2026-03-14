/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#include "device.hpp"

#include "can.h"
#include "cmsis_os2.h"

motors::DJIMotor* motor_wheel[4];

sensors::gyro::HWT101CT* sensor_gyro_yaw;

UartRxSync_DefineCallback(sensor_gyro_yaw);

static void sensor_init()
{
    using sensors::gyro::HWT101CT;

    sensor_gyro_yaw = new HWT101CT(SensorGyroYawUart);
    UartRxSync_RegisterCallback(sensor_gyro_yaw, SensorGyroYawUart);

    if (!sensor_gyro_yaw->startReceive())
        Error_Handler();
}

static void can_init()
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

static void motor_wheel_init()
{
    using motors::DJIMotor;
    for (size_t i = 0; i < 4; ++i)
        motor_wheel[i] = new DJIMotor(motor_wheel_config[i]);
}

void Device_Init()
{
    sensor_init();

    can_init();

    motor_wheel_init();
}

void Device_Update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool Device_isAllConnected()
{
    // sensor
    if (!sensor_gyro_yaw->isConnected())
        return false;

    for (auto& m : motor_wheel)
        if (!m->isConnected())
            return false;

    return true;
}
void Device_WaitAllConnected()
{
    while (!Device_isAllConnected())
        osDelay(1);
}

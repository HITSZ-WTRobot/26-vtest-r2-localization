/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-13
 * @attention requires C++ version >= C++17
 */
#pragma once
#include "HWT101CT.hpp"
#include "dji.hpp"
#include "usart.h"

namespace config::uart
{
inline UART_HandleTypeDef* SensorGyroYawUart = &huart2;
inline UART_HandleTypeDef* PCUart            = &huart3;
} // namespace config::uart

namespace Device
{

// sensors
namespace sensor
{
/**
 * Z 轴高精度陀螺仪
 */
inline sensors::gyro::HWT101CT* gyro_yaw = nullptr;
} // namespace sensor

// motors
namespace motor
{
/**
 * 底盘电机
 */
inline motors::DJIMotor* wheel[4] = { nullptr };
} // namespace motor

// functions

void init();
void update_1kHz();
bool isAllConnected();
void waitAllConnected();

} // namespace Device
/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#pragma once
#include "HWT101CT.hpp"
#include "dji.hpp"
#include "usart.h"

extern motors::DJIMotor* motor_wheel[4];

constexpr UART_HandleTypeDef*   SensorGyroYawUart = &huart2;
extern sensors::gyro::HWT101CT* sensor_gyro_yaw;

constexpr UART_HandleTypeDef* PCUart = &huart3;

void Device_Init();

void Device_Update_1kHz();

bool Device_isAllConnected();

void Device_WaitAllConnected();
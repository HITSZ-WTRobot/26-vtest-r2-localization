/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#pragma once

#include "LocEKF.hpp"
#include "Master.hpp"
#include "Mecanum4.hpp"

using Chassis = chassis::controller::Master<chassis::Mecanum4>;
extern chassis_loc::LocEKF* loc_ekf;
extern Chassis*             chassis_;

void Chassis_BeforeUpdate();

void Chassis_Init();

void Chassis_Update_1kHz();

void Chassis_Update_100Hz();
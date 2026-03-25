/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-03-13
 *
 * 底盘 + 定位系统启动步骤：
 * 1. motor & motor_ctrl 初始化，motion 初始化
 * 2. motion 校准/自检 | 接收上位机传来的初始位置
 * 3. loc 根据初始位置初始化，chassis_ctrl 初始化
 * 4. 完成
 */
#pragma once

#include "LocEKF.hpp"
#include "Master.hpp"
#include "Mecanum4.hpp"

namespace Chassis
{

using ChassisController = chassis::controller::Master;
using ChassisLoc        = chassis::loc::LocEKF;
using ChassisMotion     = chassis::motion::Mecanum4;

inline ChassisLoc*        loc   ;
inline ChassisController* ctrl  ;
inline ChassisMotion*     motion;

void motionInit();

void locCtrlInit(const chassis::Posture& init_pos);

void enable();

void update_1kHz();

void update_100Hz();

} // namespace Chassis
/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#include "chassis.hpp"
#include "system.hpp"
#include "LocEKF.hpp"
#include "device.hpp"

namespace Chassis
{
controllers::MotorVelController* motor_vel_ctrl[4] = { nullptr };

constexpr PIDMotor::Config motor_wheel_vel_pid = { //
    .Kp             = 45.0f,
    .Ki             = 0.15f,
    .Kd             = 0.00f,
    .abs_output_max = 12000.0f
};

void motionInit()
{
    using controllers::MotorVelController, chassis::motion::Mecanum4;

    for (size_t i = 0; i < 4; ++i)
        motor_vel_ctrl[i] = new MotorVelController(Device::motor::wheel[i],
                                                   { motor_wheel_vel_pid });

    motion = new Mecanum4({
            .wheel_radius      = 77.0f,   ///< 轮子半径 (unit: mm)
            .wheel_distance_x  = 748.60f, ///< 左右轮距 (unit: mm)
            .wheel_distance_y  = 500.00f, ///< 前后轮距 (unit: mm)
            .chassis_type      = Mecanum4::ChassisType::OType,
            .wheel_front_right = motor_vel_ctrl[2], ///< 右前方
            .wheel_front_left  = motor_vel_ctrl[3], ///< 左前方
            .wheel_rear_left   = motor_vel_ctrl[0], ///< 左后方
            .wheel_rear_right  = motor_vel_ctrl[1], ///< 右后方
    });
}

void locCtrlInit(const chassis::Posture& init_pos)
{
    using chassis::loc::LocEKF;

    constexpr auto sq = [](const float a) { return a * a; };

    const auto init_gyro_yaw = Device::sensor::gyro_yaw->getYaw();

    loc = new LocEKF(
            *motion,
            { .x_init = { init_pos.x, init_pos.y, init_gyro_yaw, init_pos.yaw - init_gyro_yaw },
              .covP   = { sq(0.1), sq(0.1), sq(10) },
              .noiseQ = { sq(0.05), sq(0.5), sq(0.01) },
              .noiseR = { .gyro = { sq(0.1) }, .lidar = { sq(0.01), sq(0.5) } } },
            *Device::sensor::gyro_yaw,
            1);

    ctrl = new ChassisController(*motion, *loc,
                        {
                            .posture_error_pd_cfg = {
                                .vx = { .Kp = 7.0f, .Kd = 3.8f, .abs_output_max = 3.0f },
                                .vy = { .Kp = 7.0, .Kd = 3.8f, .abs_output_max = 3.0f },
                                .wz = { .Kp = 30, .Kd = 4.0f, .abs_output_max = 25.0f },
                            },
                            .limit = {
                                .x = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 50.0f },
                                .y   = { .max_spd = 8.0f, .max_acc = 3.0f, .max_jerk = 50.0f },
                                .yaw = { .max_spd = 720, .max_acc = 360, .max_jerk = 1080 }
                            }
                        });
}

void enable()
{
    if (!ctrl->enable())
        Error_Handler();
}

static uint32_t prescaler = 0;

void update_1kHz()
{
    if (loc != nullptr && ctrl != nullptr)
    {
        loc->update();

        ++prescaler;
        if (prescaler == 2)
        {
            prescaler = 0;
            ctrl->errorUpdate();
        }

        ctrl->controllerUpdate();
    }

    motion->update();
}

void update_100Hz()
{
    if (ctrl != nullptr)
        ctrl->profileUpdate(0.01);
}
} // namespace Chassis

// 系统初始化钩子
void System::Init::initPosReceived()
{
    Chassis::locCtrlInit(pos);
}
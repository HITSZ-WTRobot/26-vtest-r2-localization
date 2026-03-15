/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-03-13
 */
#include "chassis.hpp"

#include "LocEKF.hpp"
#include "device.hpp"

ChassisController*         chassis_;
ChassisLoc*                loc_ekf;
chassis::motion::Mecanum4* mecanum4;

controllers::MotorVelController* motor_vel_ctrl[4];

constexpr PIDMotor::Config motor_wheel_vel_pid = { //
    .Kp             = 45.0f,
    .Ki             = 0.15f,
    .Kd             = 0.00f,
    .abs_output_max = 8000.0f
};

void Chassis_BeforeUpdate()
{
    using controllers::MotorVelController, chassis::loc::LocEKF, chassis::motion::Mecanum4;

    for (size_t i = 0; i < 4; ++i)
        motor_vel_ctrl[i] = new MotorVelController(motor_wheel[i], { motor_wheel_vel_pid });

    constexpr auto sq = [](const float a) { return a * a; };

    mecanum4 = new Mecanum4({
            .wheel_radius      = 77.0f,   ///< 轮子半径 (unit: mm)
            .wheel_distance_x  = 748.60f, ///< 左右轮距 (unit: mm)
            .wheel_distance_y  = 500.00f, ///< 前后轮距 (unit: mm)
            .chassis_type      = Mecanum4::ChassisType::OType,
            .wheel_front_right = motor_vel_ctrl[2], ///< 右前方
            .wheel_front_left  = motor_vel_ctrl[3], ///< 左前方
            .wheel_rear_left   = motor_vel_ctrl[0], ///< 左后方
            .wheel_rear_right  = motor_vel_ctrl[1], ///< 右后方
    });

    loc_ekf = new LocEKF(*mecanum4,
                         { .x_init = { 0, 0, 0 },
                           .covQ   = { sq(0.1), sq(0.1), sq(10) },
                           .noiseQ = { sq(0.05), sq(0.5), sq(0.01) },
                           .noiseR = { .gyro = { sq(0.1) }, .lidar = { sq(0.01), sq(0.5) } } },
                         *sensor_gyro_yaw,
                         1);

    chassis_ = new ChassisController(*mecanum4, *loc_ekf,
                        {
                            .posture_error_pd_cfg = {
                                .vx = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
                                .vy = { .Kp = 5, .Kd = 3.0f, .abs_output_max = 0.1f },
                                .wz = { .Kp = 30, .Kd = 4.0f, .abs_output_max = 25.0f },
                            },
                            .limit = {
                                .x = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 2.0f },
                                .y   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 2.0f },
                                .yaw = { .max_spd = 180, .max_acc = 45, .max_jerk = 90 }
                            }
                        });
}
void Chassis_Init()
{
    if (!chassis_->enable())
        Error_Handler();
}

static uint32_t prescaler = 0;

void Chassis_Update_1kHz()
{
    ++prescaler;
    loc_ekf->update();

    if (prescaler == 2)
    {
        prescaler = 0;
        chassis_->errorUpdate();
    }

    chassis_->controllerUpdate();
}

void Chassis_Update_100Hz()
{
    chassis_->profileUpdate(0.01);
}

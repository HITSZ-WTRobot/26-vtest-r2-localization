#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"
bool init_pos_received = false;

uint32_t pc_time;

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    Device_Update_1kHz();

    service::Watchdog::EatAll();

    pc_time = clock_->getPCTime();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim)
{
    Chassis_Update_1kHz();
}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    Chassis_Update_100Hz();
}

chassis::Posture  pos;
chassis::Velocity vel;

// 0
uint8_t s = 0;

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */

    Device_Init();

    ChassisMotion_Init();

    Protocol_Init();

    ChassisLoc_Init();

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // while (!init_pos_received)
    //     osDelay(1);

    Device_WaitAllConnected();

    osDelay(2000);

    Chassis_Init();

    using namespace chassis;
    Velocity v = vel;
    Posture  p = pos;

    for (;;)
    {
        if (s == 0)
        {
            chassis_->stop();
        }
        else if (s == 1)
        {
            if (vel.vx != v.vx || vel.vy != v.vy || vel.wz != v.wz)
            {
                v = vel;
                chassis_->setVelocityInBody(v, false);
            }
        }
        else if (s == 2)
        {
            if (pos.x != p.x || pos.y != p.y || pos.yaw != p.yaw)
            {
                p = pos;
                chassis_->setTargetPostureInWorld(p);
            }
        }

        osDelay(1);
    }

    /* 初始化完成后退出线程 */
    osThreadExit();
}
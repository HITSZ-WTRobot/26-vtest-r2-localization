#include "IChassisDef.hpp"
#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"

uint32_t now;

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    Device::update_1kHz();

    service::Watchdog::EatAll();

    now = HAL_GetTick();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim)
{
    // if (init_pos_received)
    Chassis::update_1kHz();
}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    // if (init_pos_received)
    Chassis::update_100Hz();
}

volatile bool can_run = false;

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" [[noreturn]] void Init(void* argument)
{
    /* 初始化代码 */

    Device::init();

    Chassis::motionInit();

    Protocol::init();

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    Device::waitAllConnected();

    osDelay(2000);

    // waiting for system inited
    while (!System::Init::inited())
        osDelay(1);

    osDelay(1000);

    Chassis::ctrl->enable();

    // Chassis::ctrl->setTargetPostureInWorld({3.5f, 0.0f, 0.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({3.5f, 0.0f, -90.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({3.5f, -3.5f, -90.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({3.5f, -3.5f, -180.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({0.0f, -3.5f, -180.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({0.0f, 0.0f, -180.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    // osDelay(1000);
    // Chassis::ctrl->setTargetPostureInWorld({0.0f, 0.0f, 0.0f});
    // Chassis::ctrl->waitTrajectoryFinish();
    osDelay(2000);
    // while (!can_run)
    // {
    //     osDelay(2000);
    // }
    osDelay(8000);

    auto [x, y, yaw] = System::Init::pos;

    for (size_t i = 0; i < 8; ++i)
    {
        Chassis::ctrl->setTargetPostureInWorld({ 2.0f + x, 0.0f + y, 0.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 2.0f + x, 0.0f + y, 90.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 2.0f + x, 2.0f + y, 90.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 2.0f + x, 2.0f + y, 180.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 0.0f + x, 2.0f + y, 180.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 0.0f + x, 2.0f + y, 90.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 0.0f + x, 0.0f + y, 90.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
        Chassis::ctrl->setTargetPostureInWorld({ 0.0f + x, 0.0f + y, 0.0f + yaw });
        Chassis::ctrl->waitTrajectoryFinish();
        osDelay(1000);
    }
    /* 初始化完成后退出线程 */
    osThreadExit();
}
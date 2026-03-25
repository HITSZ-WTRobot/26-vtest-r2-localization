#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    Device::update_1kHz();

    service::Watchdog::EatAll();
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

    Chassis::ctrl->enable();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
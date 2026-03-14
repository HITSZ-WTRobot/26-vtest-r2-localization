#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "protocol.hpp"
#include "tim.h"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    Device_Update_1kHz();

    service::Watchdog::EatAll();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim)
{
    Chassis_Update_1kHz();
}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    Chassis_Update_100Hz();
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */

    Device_Init();

    Chassis_BeforeUpdate();

    Protocol_Init();

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);

    osDelay(2000);

    Chassis_Init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
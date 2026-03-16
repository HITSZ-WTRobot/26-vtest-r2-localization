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

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);

    while (!init_pos_received)
        osDelay(1);

    ChassisLoc_Init();

    Device_WaitAllConnected();

    osDelay(2000);

    for (;;)
        osDelay(1);

    Chassis_Init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
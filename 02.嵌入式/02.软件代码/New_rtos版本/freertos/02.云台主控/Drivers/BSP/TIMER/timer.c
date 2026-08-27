#include "timer.h"
#include "usart.h"
#include "imu.h"
#include "nrf24l01.h"
#include "data.h"

TIM_HandleTypeDef htim2;

/**
 * @brief TIM2 初始化函数
 * @param arr 自动重装载值
 * @param psc 预分频系数
 */
void TIM2_Init(uint16_t arr, uint16_t psc)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = psc;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        // 错误处理
    }

    // 启动定时器中断
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief HAL库底层MSP回调，负责时钟使能和NVIC配置
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // 1. 使能时钟
        __HAL_RCC_TIM2_CLK_ENABLE();

        // 2. 配置NVIC优先级
        HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}
/**
  * @brief 定时器2中断服务程序 (由启动文件向量表调用)
  */
void TIM2_IRQHandler(void)
{
    // 调用HAL库通用处理函数，内部会自动清除标志位并调用回调函数
    HAL_TIM_IRQHandler(&htim2);
}
/**
 * @brief 定时器溢出中断回调函数 (业务逻辑写在这里)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
		//imu数据更新
		imu_updata();
		//双环pid计算
		pid_calculate();

//	usart3_dma_send(test_frame, sizeof(test_frame));
		
    }
}




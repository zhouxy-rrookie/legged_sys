#include "bsp_tim.h"

//TIM4是200Hz,用于发送电机指令
//TIM5是1MHz,用于微秒级延时

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        
    }
}

void delay_us(uint32_t us)
{
  uint32_t start_cnt = __HAL_TIM_GET_COUNTER(&htim5); 
  while((__HAL_TIM_GET_COUNTER(&htim5) - start_cnt) < us);
}


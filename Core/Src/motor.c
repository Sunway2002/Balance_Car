#include "motor.h"

/*
	设置电机速度
	输入: 
		speed: 电机速度（0-1000）;
		side:  选择左/右电机(Left/Right)
		
	TIM_CHANNEL_3对应电路板上的A，M2
*/
void Set_Motor_Speed(uint16_t speed, int side)
{
	if(side == Left)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, speed);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	}
	else if(side == Right)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, speed);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	}
}

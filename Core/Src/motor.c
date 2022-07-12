#include "motor.h"

/*
	���õ���ٶ�
	����: 
		speed: ����ٶȣ�0-1000��;
		side:  ѡ����/�ҵ��(Left/Right)
		
	TIM_CHANNEL_3��Ӧ��·���ϵ�A��M2
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

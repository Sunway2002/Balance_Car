#include "motor.h"

/*
	���õ��ʹ�ܶ�
	����: 
		direction: ѡ����ת��(Forward/Backward)
		side:  ѡ����/�ҵ��(Left/Right)
*/
void Direction(int direction, int side)
{
	if(direction == Forward)
	{
		if(side == Left)
		{
			HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
		}
	}
	else if(direction == Backward)
	{
		if(side == Left)
		{
			HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
		}
	}


}




/*
	���õ���ٶ�
	����: 
		speed: ����ٶȣ�0-1000��;
		side:  ѡ����/�ҵ��(Left/Right)
		
	TIM_CHANNEL_3��Ӧ����A��M2
*/
void Set_Motor_Speed(int speed, int side)
{
	int d;
	if(speed >= 0)
	{
		d = Forward;
		speed = speed;
	}
	else 
	{
		d = Backward;
		speed = -speed;
	}
	if(side == Left)
	{
		Direction(d,Left);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, speed);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	}
	else if(side == Right)
	{
		Direction(d,Right);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	}
}


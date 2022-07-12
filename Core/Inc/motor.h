#ifndef __MOTOR_H__
#define __MOTOR_H__

#define Left 0
#define Right 1

#include "tim.h"
#include "main.h"

void Set_Motor_Speed(uint16_t speed, int side);
#endif


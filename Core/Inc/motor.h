#ifndef __MOTOR_H__
#define __MOTOR_H__

#define Left 0
#define Right 1
#define Forward 2
#define Backward 3

#include "tim.h"
#include "main.h"

void Direction(int direction, int side);
void Set_Motor_Speed(int speed, int side);
#endif


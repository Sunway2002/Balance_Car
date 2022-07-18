#ifndef __MPU_H
#define __MPU_H
#include "DMP.h"

float ComplementaryFilter(float acc, float gyro, float dt);
void GetMpuData(void);
void AngleCalculate(void);
#endif

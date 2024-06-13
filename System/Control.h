#ifndef __CONTROL_H
#define __CONTROL_H

#include "Timer.h"
#include "OLED.h"
#include "IMU.h"
#include "motor.h"
#include "encoder.h"

void TIM3_IRQHandler(void);

float Vertical_FB(float Angle,float gyro_Y);
float Vertical_RL(float Angle,float gyro_X);
float Velocity_FB(void);
float Velocity_RL(void);

#endif

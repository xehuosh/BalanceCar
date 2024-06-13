#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "OLED.h"
#include "IMU.h"
#include "Key.h"
#include "motor.h"
#include "encoder.h"
#include "Control.h"
#include "MPU6050.h"

int main(void)
{
	OLED_Init();
	Key_Init();
	MPU6050_Init();
	Timer_Init();
	Motor_Init(1 - 1, 7200 - 1);
	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	Load(0, 0);
	while (1)
	{
		//if(Key_GetNum() == 1)
		OLED_ShowString(1, 1, "pitch:");
		OLED_ShowString(2, 1, "roll :");
		OLED_ShowString(3, 1, "yaw  :");
		OLED_ShowFloatNum(1, 7, imu_Angle.Pitch, 5);
		OLED_ShowFloatNum(2, 7, imu_Angle.Roll , 5);
		OLED_ShowFloatNum(3, 7, imu_Angle.Yaw  , 5);
	}
}

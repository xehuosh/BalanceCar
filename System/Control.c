#include "stm32f10x.h"                  // Device header
#include "Control.h"

#define cycle_T            0.005f       //200Hz
#define filter_weight      0.3f
#define max_tilt_angle     30.0f

//parameters of PD controller for the motor located below
#define Vertical_FB_Kp    -810
#define Vertical_FB_Kd    -2700

//parameters of PI controller for the motor located below
#define Velocity_FB_Kp    -1000
#define Velocity_FB_Ki     0

//parameters of PD controller for the motor located above
#define Vertical_RL_Kp    -1850
#define Vertical_RL_Kd    -150

//parameters of PI controller for the motor located above
#define Velocity_RL_Kp     0
#define Velocity_RL_Ki     0

//mechanical neutral angle
#define Med_Angle_FB       3.3
#define Med_Angle_RL       0

//motor target speed (encoder)
#define Target_Speed_FB    0
#define Target_Speed_RL    0

static float last_filt_FB_velocity;
static float last_filt_RL_velocity;
static float filt_FB_velocity;
static float filt_RL_velocity;
static float FB_velocity_sum;
static float RL_velocity_sum;

float Vertical_FB(float Angle, float gyro_Y);
float Vertical_RL(float Angle, float gyro_X);
float Velocity_FB(void);
float Velocity_RL(void);

float angle_abs(float angle)
{
	return angle>0?angle:(-angle);
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		IMU_getEuleranAngles();
		int PWM_U = -Vertical_RL(imu_Angle.Pitch, imu_data.GX) - Velocity_RL();
		int PWM_D =  Vertical_FB(imu_Angle.Roll , imu_data.GY) + Velocity_FB();
		Motor_Xianfu(7200, &PWM_U, &PWM_D);
		
		if(angle_abs(imu_Angle.Pitch) < max_tilt_angle && angle_abs(imu_Angle.Roll) < max_tilt_angle)
			Load(PWM_U, PWM_D);
		else
		{
			Load(0, 0);
			FB_velocity_sum = 0;
			RL_velocity_sum = 0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

float Vertical_FB(float Roll , float gyro_Y)
{
	return Vertical_FB_Kp * (Roll  - Med_Angle_FB) + Vertical_FB_Kd * gyro_Y * cycle_T;
}

float Vertical_RL(float Pitch, float gyro_X)
{
	return Vertical_RL_Kp * (Pitch - Med_Angle_RL) + Vertical_RL_Kd * gyro_X * cycle_T;
}

void Integral_Xianfu(int Int_MAX, float *Param)
{
	if(*Param >  Int_MAX) *Param =  Int_MAX;
	if(*Param < -Int_MAX) *Param = -Int_MAX;
}

float Velocity_FB(void)
{
	float FB_velocity = Read_Speed(2) * 1.0f;
	filt_FB_velocity = filter_weight * FB_velocity + (1 - filter_weight) * last_filt_FB_velocity;
	FB_velocity_sum +=  filt_FB_velocity;
	Integral_Xianfu(3000, &FB_velocity_sum);
	last_filt_FB_velocity = filt_FB_velocity;
	return Velocity_FB_Kp * (filt_FB_velocity - Target_Speed_FB) + Velocity_FB_Ki * FB_velocity_sum;
}

float Velocity_RL(void)
{
	float RL_velocity = Read_Speed(4) * 1.0f;
	filt_RL_velocity = filter_weight * RL_velocity + (1 - filter_weight) * last_filt_RL_velocity;
	RL_velocity_sum +=  filt_RL_velocity;
	Integral_Xianfu(3000, &RL_velocity_sum);
	last_filt_RL_velocity = filt_RL_velocity;
	return Velocity_RL_Kp * (filt_RL_velocity - Target_Speed_RL) + Velocity_RL_Ki * RL_velocity_sum;
}

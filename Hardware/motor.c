#include "stm32f10x.h"                  // Device header
#include "pwm.h"

void Motor_Init(uint16_t Psc, uint16_t Per)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	PWM_Init(Psc, Per);
}

void Motor_Xianfu(int PWM_MAX, int *PWM_U, int *PWM_D)
{
	if(*PWM_U > PWM_MAX) *PWM_U = PWM_MAX;
	if(*PWM_U <-PWM_MAX) *PWM_U =-PWM_MAX;
	
	if(*PWM_D > PWM_MAX) *PWM_D = PWM_MAX;
	if(*PWM_D <-PWM_MAX) *PWM_D =-PWM_MAX;
}

void Load(int PWM_U, int PWM_D)
{		
	if(PWM_U >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		TIM_SetCompare4(TIM1, 7200 - PWM_U);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		TIM_SetCompare4(TIM1, 7200 + PWM_U);
	}
	
	if (PWM_D >= 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		TIM_SetCompare1(TIM1, PWM_D + 500);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		TIM_SetCompare1(TIM1,-PWM_D + 500);
	}
}

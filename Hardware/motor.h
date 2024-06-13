#ifndef  _MOTOR_H
#define  _MOTOR_H

void Motor_Init(uint16_t Psc, uint16_t Per);
void Motor_Xianfu(int PWM_MAX, int *PWM_U, int *PWM_D);
void Load(int PWM_U, int PWM_D);

#endif

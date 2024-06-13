#include "stm32f10x.h"                  // Device header
#include "IMU.h"
#include "MPU6050.h"

#define Mohony_Kp      20.00f
#define Mohony_Ki      0.001f
#define cycle_T        0.005f      //200Hz
#define half_T         0.0025f
#define pi             3.1415926f

int16_t Ax, Ay, Az, Gx, Gy, Gz;
param_imu imu_data;
param_Angle imu_Angle;

float q[4] = {1.0,0.0,0.0,0.0};
float exInt = 0.0,eyInt = 0.0,ezInt = 0.0;

float fast_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *) &y;
	i = 0x5f3759df - (i>>1);
	y = *(float *) &i;
	y = y * (1.5f-(halfx * y * y));
	return y;
}

void IMU_GetValues(void)
{
	MPU6050_GetData(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
	//Full Scale +-16 g
	imu_data.AX = ((float)Ax)*32/65536;
	imu_data.AY = ((float)Ay)*32/65536;
	imu_data.AZ = ((float)Az)*32/65536;
	//Full Scale +-2000 degree/s
	imu_data.GX = ((float)Gx)*4000/65536/180*pi;
	imu_data.GY = ((float)Gy)*4000/65536/180*pi;
	imu_data.GZ = ((float)Gz)*4000/65536/180*pi;
}

void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
	float vx,vy,vz;
	float ex,ey,ez;
	ax = imu_temp->AX;
	ay = imu_temp->AY;
	az = imu_temp->AZ;
	gx = imu_temp->GX;
	gy = imu_temp->GY;
	gz = imu_temp->GZ;
	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];
	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q2*q3 + q0*q1);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;
	exInt += Mohony_Ki * ex;
	eyInt += Mohony_Ki * ey;
	ezInt += Mohony_Ki * ez;
	gx += Mohony_Kp * ex + exInt;
	gy += Mohony_Kp * ey + eyInt;
	gz += Mohony_Kp * ez + ezInt;
	q0 += (-q1*gx-q2*gy-q3*gz) * half_T;
	q1 += ( q0*gx+q2*gz-q3*gy) * half_T;
	q2 += ( q0*gy-q1*gz+q3*gx) * half_T;
	q3 += ( q0*gz+q1*gy-q2*gx) * half_T;
	norm = fast_sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;
}

void IMU_getEuleranAngles(void)
{
	IMU_GetValues();
	IMU_AHRSupdate(&imu_data);
	imu_Angle.Pitch = asin(-2*q[1]*q[3]+2*q[0]*q[2])*180/pi;
	imu_Angle.Roll  = atan2(2*q[2]*q[3]+2*q[0]*q[1],-2*q[1]*q[1]-2*q[2]*q[2]+1)*180/pi;
	//If "imu_data.GZ" is too small, it is considered to be noise interference.(1 degree/s)
	if((imu_data.GZ*180/pi > 1.0f) || (imu_data.GZ*180/pi < -1.0f))
	{
		imu_Angle.Yaw  += imu_data.GZ*cycle_T*4*180/pi;
	}
}

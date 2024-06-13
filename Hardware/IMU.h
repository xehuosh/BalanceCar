#ifndef __IMU_H
#define __IMU_H

#include "math.h"

typedef struct{
		float AX;
		float AY;
		float AZ;
		float GX;
		float GY;
		float GZ;
}param_imu;

typedef struct{
		float Pitch;
		float Roll;
		float Yaw;
}param_Angle;

extern param_imu imu_data;
extern param_Angle imu_Angle;
void IMU_getEuleranAngles(void);

#endif

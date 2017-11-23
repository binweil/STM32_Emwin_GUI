#ifndef __DMP_H              //∑¿÷π÷ÿ∏¥define
#define __DMP_H

float GetRoll(float aacx,float aacy,float aacz);
float GetPitch(float aacx,float aacy,float aacz);
float GetYaw(float gyroz);

float Kalman_Filter_X(float accel,float gyro);
float Kalman_Filter_Y(float accel,float gyro);
float Kalman_Filter_Z(float accel,float gyro);

#endif


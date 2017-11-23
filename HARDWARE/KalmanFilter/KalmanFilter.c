#include "KalmanFilter.h"

float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;
float angle = 0.0f; // Reset the angle
float bias = 0.0f; // Reset bias
float P_00 = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
float P_01 = 0.0f;
float P_10= 0.0f;
float P_11= 0.0f;
float rate;


float KalFilter(float newAngle, float newRate,float dt)
{
	rate = newRate - bias;
	angle += dt * rate;
	
	P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
  P_01 -= dt * P_11;
  P_10 -= dt * P_11;
  P_11 += Q_bias * dt;
	
	float S=P_00+R_measure;
	
	float K_0,K_1;
	K_0=P_00/S;
	K_1=P_10/S;
	
	float y=newAngle-angle;
	
	angle+=K_0*y;
	bias +=K_1*y;
	
	float P00_temp=P_00;
	float P01_temp=P_01;
	
	  P_00 -= K_0 * P00_temp;
    P_01 -= K_0 * P01_temp;
    P_10 -= K_1 * P00_temp;
    P_11 -= K_1 * P01_temp;

    return angle;
	
}



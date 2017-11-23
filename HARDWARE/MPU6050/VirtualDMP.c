#include "sys.h"
#include "usart.h"
#include "VirtualDMP.h"
#include "MPU6050.h"
#include <math.h>

/************************************************从滤波完的原始数据得到Roll(x_axis)*********************************************/
float GetRoll(float aacx,float aacy,float aacz)
{
	float Roll;
	float normal=sqrt(aacx*aacx+aacy*aacy+aacz*aacz);
	Roll=acos(sqrt(aacx*aacx+aacz*aacz)/normal)*57.3;
	return Roll;
}

/*************************************************从滤波完的原始数据得到Pitch(y_axis)*********************************************/
float GetPitch(float aacx,float aacy,float aacz)
{
	float Pitch;
	float normal=sqrt(aacx*aacx+aacy*aacy+aacz*aacz);
	Pitch=acos(sqrt(aacy*aacy+aacz*aacz)/normal)*57.3;
	return Pitch;
}

/***************************************z轴Yaw角速度积分法********************************************************/
float Yaw;
float acceleration;
int gyroz_last;
float GetYaw(float gyroz)
{
	float dt=0.00414;//根据每个loop的具体时间进行赋值
	/******************************为了去除噪音而用加速度做的低通滤波，积分步骤融合进去了********************************************/
	acceleration=(gyroz-gyroz_last)/dt;							
	//printf("gyrozacc is \t%0.1f\t",acceleration); //可以看噪音的大小
	if(acceleration >=242 || acceleration <=-242)   //+-242是噪音
	{
	Yaw+= gyroz*dt*180/590; //积分角度并且按测试到的数据的比例转换
	}
	gyroz_last=gyroz;
	
	return Yaw;
}

/**********************************************x轴的卡尔曼滤波******************/

static float dt=0.5;
float Q_accex=0.9,Q_gyrox=0.001, R_anglex=10;  //各个轴要分开定义function
float H_0x=1;  
static float PPx[2][2]={1,0,0,1};
float Pdotx[4];  
float angle_errorx;  
float gyro_biasx=0;
float angle_x, rotate_x;
float PHt_0,PHt_1,E,K_0,K_1,Y_0,Y_1;  
float Kalman_Filter_X(float accel,float gyro)
{        
   angle_x+=(gyro-gyro_biasx)*dt;  
   Pdotx[0]=Q_accex-PPx[0][1]-PPx[1][0];  
   Pdotx[1]=-PPx[1][1];  
   Pdotx[2]=-PPx[1][1];  
   Pdotx[3]=Q_gyrox;  
          
   PPx[0][0]+=Pdotx[0]*dt;  
   PPx[0][1]+=Pdotx[1]*dt;  
   PPx[1][0]+=Pdotx[2]*dt;  
   PPx[1][1]+=Pdotx[3]*dt;  
        
   PHt_0=H_0x*PPx[0][0];  
   PHt_1=H_0x*PPx[1][0];  
        
   E=R_anglex+H_0x*PHt_0; 
        
   K_0=PHt_0/E;  
   K_1=PHt_1/E;  
        
   angle_errorx=accel-angle_x;  
   angle_x+=K_0*angle_errorx;
   gyro_biasx+=K_1*angle_errorx;  
   rotate_x=gyro-gyro_biasx;
       
   Y_0=PHt_0;  
   Y_1=H_0x*PPx[0][1];  
          
   PPx[0][0]-=K_0*Y_0;  
   PPx[0][1]-=K_0*Y_1;  
   PPx[1][0]-=K_1*Y_0;  
   PPx[1][1]-=K_1*Y_1;
	return angle_x;	
}

/*****************************************y轴的卡尔曼滤波*************************************************/

float Q_accey=0.8,Q_gyroy=0.001, R_angley=10;  //R为0的话数据不会串 但是没有效果了
float H_0y=1;  
static float PPy[2][2]={1,0,0,1};
float Pdoty[4];  
float angle_errory;  
float gyro_biasy=0;
float angle_y, rotate_y;	
float PHt_0y,PHt_1y,Ey,K_0y,K_1y,Y_0y,Y_1y; 
float Kalman_Filter_Y(float accel,float gyro)
{     
   angle_x+=(gyro-gyro_biasy)*dt;  
   Pdoty[0]=Q_accey-PPy[0][1]-PPy[1][0];  
   Pdoty[1]=-PPy[1][1];  
   Pdoty[2]=-PPy[1][1];  
   Pdoty[3]=Q_gyroy;  
          
   PPy[0][0]+=Pdoty[0]*dt;  
   PPy[0][1]+=Pdoty[1]*dt;  
   PPy[1][0]+=Pdoty[2]*dt;  
   PPy[1][1]+=Pdoty[3]*dt;  
        
   PHt_0y=H_0y*PPy[0][0];  
   PHt_1y=H_0y*PPy[1][0];  
        
   Ey=R_angley+H_0y*PHt_0y; 
        
   K_0y=PHt_0y/Ey;  
   K_1y=PHt_1y/Ey;  
        
   angle_errory=accel-angle_y;  
   angle_y+=K_0y*angle_errory;
   gyro_biasy+=K_1y*angle_errory;  
   rotate_y=gyro-gyro_biasy;    //angular velocity best estimate
       
   Y_0y=PHt_0y;  
   Y_1y=H_0y*PPy[0][1];  
          
   PPy[0][0]-=K_0y*Y_0y;  
   PPy[0][1]-=K_0y*Y_1y;  
   PPy[1][0]-=K_1y*Y_0y;  
   PPy[1][1]-=K_1y*Y_1y;
	return angle_y;	
}

/**********************************Z轴的卡尔曼滤波**************************************************************/
float Q_accez=0,Q_gyroz=1, R_anglez=10;  //由于没有加速计的修正，Q_accez不需要了
float H_0z=1;  
static float PPz[2][2]={1,0,0,1};
float Pdotz[4];  
float angle_errorz;  
float gyro_biasz=0;
float angle_z, rotate_z;	
float PHt_0z,PHt_1z,Ez,K_0z,K_1z,Y_0z,Y_1z;  
float Kalman_Filter_Z(float accel,float gyro)
{    
   angle_z+=(gyro-gyro_biasz)*dt;  
   Pdotz[0]=Q_accez-PPz[0][1]-PPz[1][0];  
   Pdotz[1]=-PPz[1][1];  
   Pdotz[2]=-PPz[1][1];  
   Pdotz[3]=Q_gyroz;  
          
   PPz[0][0]+=Pdotz[0]*dt;  
   PPz[0][1]+=Pdotz[1]*dt;  
   PPz[1][0]+=Pdotz[2]*dt;  
   PPz[1][1]+=Pdotz[3]*dt;  
        
   PHt_0z=H_0z*PPz[0][0];  
   PHt_1z=H_0z*PPz[1][0];  
        
   Ez=R_anglez+H_0z*PHt_0z; 
        
   K_0z=PHt_0z/Ez;  
   K_1z=PHt_1z/Ez;  
        
   angle_errorz=accel-angle_z;  
   angle_z+=K_0*angle_errorz;
   gyro_biasz+=K_1*angle_errorz;  
   rotate_z=gyro-gyro_biasz;    //angular velocity best estimate
       
   Y_0z=PHt_0z;  
   Y_1z=H_0z*PPz[0][1];  
          
   PPz[0][0]-=K_0z*Y_0z;  
   PPz[0][1]-=K_0z*Y_1z;  
   PPz[1][0]-=K_1z*Y_0z;  
   PPz[1][1]-=K_1z*Y_1z;
	return angle_z;	
}


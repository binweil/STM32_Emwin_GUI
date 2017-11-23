#include "sys.h"
#include "delay.h"
#include "usart.h"  
#include "MPU6050.h"
#include "VirtualDMP.h"
#include "key.h"
//以下函数return 0是设置成功，其他设置失败，为此页最后的MPU初始化函数做准备


//函数1.向MPU6050发送一个字节，其中用到了正点原子的IIC的驱动，按F12查看详细
//IIC的驱动也可用stm32官方的，但是他们对寄存器的操作都是一样的
u8 MPU_Write_Byte(u8 reg,u8 data)	 
{ 
  IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}

u8 MPU_Set_Gyro_Fsr(u8 fsr)     //设置gyro范围0，+-250,....DPS
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//操作寄存器 
}

u8 MPU_Set_Accel_Fsr(u8 fsr)    //设置加速计范围0g,2g,.....
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//操作相应寄存器
}

u8 MPU_Set_LPF(u16 lpf)       //设置Low Pass Filter
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

u8 MPU_Set_Rate(u16 rate)    //设置采样率
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


u8 MPU_Read_Byte(u8 reg)   //读一个字节的数据
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)   //读整个数据
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}


u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)     //写整个数据长度
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 



short MPU_Get_Temperature(void)    //得到温度值
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)   //得到原始加速度数值
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz) //得到gyro的原始数据
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}




u8 MPU_Init(void)
{ 
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//speed 50hz
 	}else return 1;
	return 0;
}

void MPU_GetMeasure(void)//float *roll, float *pitch, float *yaw)
{
	float Pitch,Roll,Yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	u8 key;
	volatile short temp;					//温度
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	MPU_Init();					//初始化MPU6050
	delay_ms(100);
	while(1)
	{
		key=KEY_Scan(0);
		if (key==1)return;
		/*********************************刷新原始数据********************************************/
		temp=MPU_Get_Temperature();	//temperature original data(temp)
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//Accx original data (aacx,y,z)
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//angular velocity original data(gyrox,y,z)
		/*********************************卡尔曼滤波x,y轴数据*************************************/
		aacx=Kalman_Filter_X(aacx,gyrox);
		aacy=Kalman_Filter_Y(aacy,gyroy);
		//aacz=Kalman_Filter_Z(aacz,gyroz);//由于z轴没有参考轴不能用次滤波
		
		/**********************************用滤波完的数据计算Roll Pitch Yaw***********************/
		Roll = GetRoll(aacx,aacy,aacz);
		Pitch = GetPitch(aacx,aacy,aacz);
		Yaw = GetYaw(gyroz);
		
		/**********************************USB串口显示三个角度******************************************/
		printf("Roll is %0.1f\t",Roll);
		printf("Pitch is %0.1f\t",Pitch);
		printf("Yaw is%0.1f\r\n",Yaw);
	}
	
}


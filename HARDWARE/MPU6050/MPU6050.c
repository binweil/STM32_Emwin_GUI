#include "sys.h"
#include "delay.h"
#include "usart.h"  
#include "MPU6050.h"
#include "VirtualDMP.h"
#include "key.h"
//���º���return 0�����óɹ�����������ʧ�ܣ�Ϊ��ҳ����MPU��ʼ��������׼��


//����1.��MPU6050����һ���ֽڣ������õ�������ԭ�ӵ�IIC����������F12�鿴��ϸ
//IIC������Ҳ����stm32�ٷ��ģ��������ǶԼĴ����Ĳ�������һ����
u8 MPU_Write_Byte(u8 reg,u8 data)	 
{ 
  IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}

u8 MPU_Set_Gyro_Fsr(u8 fsr)     //����gyro��Χ0��+-250,....DPS
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//�����Ĵ��� 
}

u8 MPU_Set_Accel_Fsr(u8 fsr)    //���ü��ټƷ�Χ0g,2g,.....
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//������Ӧ�Ĵ���
}

u8 MPU_Set_LPF(u16 lpf)       //����Low Pass Filter
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

u8 MPU_Set_Rate(u16 rate)    //���ò�����
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}


u8 MPU_Read_Byte(u8 reg)   //��һ���ֽڵ�����
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)   //����������
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}


u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)     //д�������ݳ���
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 



short MPU_Get_Temperature(void)    //�õ��¶�ֵ
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)   //�õ�ԭʼ���ٶ���ֵ
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

u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz) //�õ�gyro��ԭʼ����
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
	IIC_Init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//speed 50hz
 	}else return 1;
	return 0;
}

void MPU_GetMeasure(void)//float *roll, float *pitch, float *yaw)
{
	float Pitch,Roll,Yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	u8 key;
	volatile short temp;					//�¶�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	MPU_Init();					//��ʼ��MPU6050
	delay_ms(100);
	while(1)
	{
		key=KEY_Scan(0);
		if (key==1)return;
		/*********************************ˢ��ԭʼ����********************************************/
		temp=MPU_Get_Temperature();	//temperature original data(temp)
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//Accx original data (aacx,y,z)
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//angular velocity original data(gyrox,y,z)
		/*********************************�������˲�x,y������*************************************/
		aacx=Kalman_Filter_X(aacx,gyrox);
		aacy=Kalman_Filter_Y(aacy,gyroy);
		//aacz=Kalman_Filter_Z(aacz,gyroz);//����z��û�вο��᲻���ô��˲�
		
		/**********************************���˲�������ݼ���Roll Pitch Yaw***********************/
		Roll = GetRoll(aacx,aacy,aacz);
		Pitch = GetPitch(aacx,aacy,aacz);
		Yaw = GetYaw(gyroz);
		
		/**********************************USB������ʾ�����Ƕ�******************************************/
		printf("Roll is %0.1f\t",Roll);
		printf("Pitch is %0.1f\t",Pitch);
		printf("Yaw is%0.1f\r\n",Yaw);
	}
	
}


#include "SPITest.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "ILI93xx.c"
#include "spi.h"
#include "w25qxx.h"
#include "key.h" 


/*
const u8 TEXT_Buffer[]={"Explorer STM32F4 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)	 
	
int main(void)
{ 
	u8 key;
	u16 i=0;
	u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//������ʼ��  
	W25QXX_Init();			//W25QXX��ʼ��
 	//POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"SPI TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
	LCD_ShowString(30,130,200,16,16,"KEY1:Write  KEY0:Read");	//��ʾ��ʾ��Ϣ		
	while(W25QXX_ReadID()!=W25Q128)								//��ⲻ��W25Q128
	{
		LCD_ShowString(30,150,200,16,16,"W25Q128 Check Failed!");
		delay_ms(500);
		LCD_ShowString(30,150,200,16,16,"Please Check!      ");
		delay_ms(500);
		LED0=!LED0;		//DS0��˸
	}
	LCD_ShowString(30,150,200,16,16,"W25Q128 Ready!"); 
	FLASH_SIZE=16*1024*1024;	//FLASH ��СΪ16�ֽ�
  //	POINT_COLOR=BLUE;			//��������Ϊ��ɫ	  
	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY1_PRES)//KEY1����,д��W25Q128
		{
			LCD_Fill(0,170,239,319,WHITE);//�������    
 			LCD_ShowString(30,170,200,16,16,"Start Write W25Q128....");
			W25QXX_Write((u8*)TEXT_Buffer,FLASH_SIZE-100,SIZE);		//�ӵ�����100����ַ����ʼ,д��SIZE���ȵ�����
			LCD_ShowString(30,170,200,16,16,"W25Q128 Write Finished!");	//��ʾ�������
		}
		if(key==KEY0_PRES)//KEY0����,��ȡ�ַ�������ʾ
		{
 			LCD_ShowString(30,170,200,16,16,"Start Read W25Q128.... ");
			W25QXX_Read(datatemp,FLASH_SIZE-100,SIZE);					//�ӵ�����100����ַ����ʼ,����SIZE���ֽ�
			LCD_ShowString(30,170,200,16,16,"The Data Readed Is:   ");	//��ʾ�������
			LCD_ShowString(30,190,200,16,16,datatemp);					//��ʾ�������ַ���
		} 
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0=!LED0;//��ʾϵͳ��������	
			i=0;
		}		   
	}       
}
*/


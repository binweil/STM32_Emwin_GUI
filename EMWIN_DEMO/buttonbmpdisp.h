#ifndef _BUTTONBMP_DISP_H
#define _BUTTONBMP_DISP_H
#include "sys.h"
#include "WM.h"
#include "GUI.h"
#include "DIALOG.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//STemWin BUTTONλͼ��ʾ
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/4/10
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
WM_HWIN CreateWindow2(void);
static void _cbAPPDialog(WM_MESSAGE * pMsg); 
void Buttonbmp_Demo(void);
#endif

/***********************************************************************************
  Filename:     uart.h

  Description:  ����ͷ�ļ�

***********************************************************************************/

#ifndef __UART_H
#define __UART_H
/*********************************************ͷ�ļ�����******************************************/
#include "delay.h"

typedef enum{
	UART_BAUD_115200 = 1, 	// 115200������
	UART_BAUD_38400 = 2 	// 38400������
}uartBaud;


/*********************************************��������******************************************/
void Uart1_Init(u8 baud,u8 isInt);
void Uart1_SendString(u8 *Date,int len);
void Uart1_SendByte(uchar n);
void Uart1_SendData();
void Uart1_SendData_Test(u8 i);
u8 Uart_CheckSum(u8 *pData);

#endif
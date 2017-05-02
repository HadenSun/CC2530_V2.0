/***********************************************************************************
  Filename:     uart.h

  Description:  串口头文件

***********************************************************************************/

#ifndef __UART_H
#define __UART_H
/*********************************************头文件包含******************************************/
#include "delay.h"

typedef enum{
	UART_BAUD_115200 = 1, 	// 115200波特率
	UART_BAUD_38400 = 2 	// 38400波特率
}uartBaud;


/*********************************************函数声明******************************************/
void Uart1_Init(u8 baud,u8 isInt);
void Uart1_SendString(u8 *Date,int len);
void Uart1_SendByte(uchar n);
void Uart1_SendData();
void Uart1_SendData_Test(u8 i);
u8 Uart_CheckSum(u8 *pData);

#endif
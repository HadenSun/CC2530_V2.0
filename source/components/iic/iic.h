/***********************************************************************************
  Filename:     iic.h

  Description:  iic操作头文件

***********************************************************************************/


#ifndef _IIC_H
#define _IIC_H
/*********************************************头文件包含******************************************/
#include <ioCC2530.h>
#include "delay.h"



/************************************************常量定义****************************************/
#define IIC_SCL P1_3
#define IIC_SDA P1_2
#define IIC_SCL_PORT 1
#define IIC_SCL_PIN  3
#define IIC_SDA_PORT 1
#define IIC_SDA_PIN  2




/************************************************函数声明****************************************/
void IIC_Init(void);                      //IIC初始化
void IIC_Start(void);                     //IIC开始信号
void IIC_Stop(void);                      //IIC结束信号
uchar IIC_Wait_Ack(void);                 //IIC等待回应
uchar IIC_Read_Byte(unsigned char ack);   //IIC读取一字节
void IIC_Send_Byte(uchar txd);            //IIC发送一字节
void SDA_IN();                            //IIC_SDA 输入
void SDA_OUT();                           //IIC_SDA 输出


#endif


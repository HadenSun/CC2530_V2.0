/***********************************************************************************
  Filename:     iic.h

  Description:  iic����ͷ�ļ�

***********************************************************************************/


#ifndef _IIC_H
#define _IIC_H
/*********************************************ͷ�ļ�����******************************************/
#include <ioCC2530.h>
#include "delay.h"



/************************************************��������****************************************/
#define IIC_SCL P1_3
#define IIC_SDA P1_2
#define IIC_SCL_PORT 1
#define IIC_SCL_PIN  3
#define IIC_SDA_PORT 1
#define IIC_SDA_PIN  2




/************************************************��������****************************************/
void IIC_Init(void);                      //IIC��ʼ��
void IIC_Start(void);                     //IIC��ʼ�ź�
void IIC_Stop(void);                      //IIC�����ź�
uchar IIC_Wait_Ack(void);                 //IIC�ȴ���Ӧ
uchar IIC_Read_Byte(unsigned char ack);   //IIC��ȡһ�ֽ�
void IIC_Send_Byte(uchar txd);            //IIC����һ�ֽ�
void SDA_IN();                            //IIC_SDA ����
void SDA_OUT();                           //IIC_SDA ���


#endif


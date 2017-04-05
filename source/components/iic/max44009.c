/***********************************************************************************
  Filename:     MAX44009.c

  Description:  ���մ���������

***********************************************************************************/


/*********************************************ͷ�ļ�����******************************************/
#include "max44009.h"
#include "iic.h"



/************************************************��������****************************************/


/*************************************************************************************************
* ���� ��   MAX_ReadOriginalData() => ��ȡ��������
* ���� �� 
*************************************************************************************************/
u8 MAX_ReadOriginalData(u8 *RST_H,u8 *RST_L)
{
  u8 error = 0;

  IIC_Start();
  IIC_Send_Byte(MAX_I2C_ADR_W);
  error |= IIC_Wait_Ack();
  IIC_Send_Byte(LUX_H);
  error |= IIC_Wait_Ack();
  
  IIC_Start();
  IIC_Send_Byte(MAX_I2C_ADR_R);
  error |= IIC_Wait_Ack();
  *RST_H = IIC_Read_Byte(0);
  
  IIC_Start();
  IIC_Send_Byte(MAX_I2C_ADR_W);
  error |= IIC_Wait_Ack();
  IIC_Send_Byte(LUX_L);
  error |= IIC_Wait_Ack();
  
  IIC_Start();
  IIC_Send_Byte(MAX_I2C_ADR_R);
  error |= IIC_Wait_Ack();
  *RST_L = IIC_Read_Byte(0);
  
  IIC_Stop();
  
  
  

  return error;
}
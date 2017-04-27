/***********************************************************************************
  Filename:     MAX44009.c

  Description:  光照传感器代码

***********************************************************************************/


/*********************************************头文件包含******************************************/
#include "max44009.h"
#include "iic.h"
#include <math.h>



/************************************************常量定义****************************************/


/*************************************************************************************************
* 函数 ：   MAX_ReadOriginalData() => 读取测量数据
* 描述 ： 
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


/*************************************************************************************************
* 函数 ：   MAX_ReadLightData() => 读取真实光照强度
* 描述 ： 
*************************************************************************************************/
u8 MAX_ReadLightData(u8 *RST_H,u8 *RST_L)
{
  u8 error = 0;
  u8 e = 0;
  u8 m = 0;
  u8 h,l;
  u16 r;
  float rst;
  
  error |= MAX_ReadOriginalData(&h,&l);
  e = (h >> 4) & 0x0F;
  m = (h & 0x0F) << 4 | (l & 0x0F);
  rst = 0.045f * pow(2,e) * (float)m;
    
  r = (u16)rst;
  *RST_H = (r >> 8) & 0xFF;
  *RST_L = r & 0xFF;
  
  return error;
}
/***********************************************************************************
  Filename:     sht1x.c

  Description:  温湿度传感器

***********************************************************************************/

/*********************************************头文件包含******************************************/
#include "sht1x.h"
#include "delay.h"
#include "uart.h"

#include <hal_mcu.h>


/************************************************常量定义****************************************/
const u16 SHT1x_POLYNOMIAL = 0x131;		//P(x)=x^8+x^5+x^4+1=100110001




/*************************************************************************************************
* 函数 ：   SHT1x_TransStart() => 开始采集
* 描述 ： generates a transmission start
*       _____         ________
* DATA:      |_______|
*           ___     ___
* SCK : ___|   |___|   |______
*************************************************************************************************/
void SHT1x_TransStart()
{
  SDA_OUT();
  IIC_SDA = 1;
  IIC_SCL = 0;
  halMcuWaitUs(4);
  IIC_SCL = 1;
  halMcuWaitUs(4);
  IIC_SDA = 0;
  halMcuWaitUs(4);
  IIC_SCL = 0;
  halMcuWaitUs(4);
  halMcuWaitUs(4);
  halMcuWaitUs(4);
  IIC_SCL = 1;
  halMcuWaitUs(4);
  IIC_SDA = 1;
  halMcuWaitUs(4);
  IIC_SCL = 0;
}

/*************************************************************************************************
* 函数 ：SHT1x_ReConnect() => 重新连接
* 说明 ：至少维持9个 SCL 周期的 SDA 高电平
*       _____________________________________________________         ________
* DATA:                                                      |_______|
*          _    _    _    _    _    _    _    _    _        ___     ___
* SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
*************************************************************************************************/
void SHT1x_ReConnect(void)
{
  uchar i;
  IIC_SDA = 1;
  IIC_SCL = 0;
  for(i = 0;i < 9; i++)
  {
    IIC_SCL = 1;
    halMcuWaitUs(4);
    IIC_SCL = 0;
    halMcuWaitUs(4);
  }
  SHT1x_TransStart();
}

/*************************************************************************************************
* 函数 : SHT1x_WriteByte(u8 value) => 写字节数据
* 参数 : value - 写入数据
*************************************************************************************************/
u8 SHT1x_WriteByte(u8 value)
{
  uchar i,error = 0;

  for(i = 0x80; i > 0; i/=2)
  {
    if(i & value) IIC_SDA = 1;
    else IIC_SDA = 0;
    IIC_SCL = 1;
    halMcuWaitUs(4);
    IIC_SCL = 0;
    halMcuWaitUs(4);
  }
  error = IIC_Wait_Ack();

  return error;
}

/*************************************************************************************************
* 函数 ： SHT1x_ReadByte(u8 ack) => 读取数据
* 参数 ： ack=1 应答，ack=0 无应答
* 返回 ： 读取到的数据
*************************************************************************************************/
u8 SHT1x_ReadByte(u8 ack)
{
  u8 i,val = 0;
  IIC_SDA = 1;
  SDA_IN();

  for(i = 0x80; i>0; i/=2)
  {
    IIC_SCL = 1;
    if(IIC_SDA) val = (val | i);
    halMcuWaitUs(4);
    IIC_SCL = 0;
    halMcuWaitUs(4);
  }

  SDA_OUT();

  if(ack == 1) IIC_SDA = 0;
  else IIC_SDA = 1;
  halMcuWaitUs(4);
  IIC_SCL = 1;
  halMcuWaitUs(4);
  IIC_SCL = 0;
  halMcuWaitUs(4);
  IIC_SDA = 1;
  return val;
}

/*************************************************************************************************
* 函数 : SHT1x_ReadResult() => 采集温湿度信息
* 参数 : * p_value 采集到的数据
        * p_checksum 校验信息
        mode 温度，湿度
* 返回 ：采集状态
*************************************************************************************************/
u8 SHT1x_ReadResult(u16 *p_value, u8 *p_checksum, u8 mode)
{
  u8 error = 0;
  u16 i;

  SHT1x_TransStart();
  switch(mode)
  {
    case SHT1x_HUMIDITY:
      error |= SHT1x_WriteByte(SHT1x_RH_MEASUREMENT);
      break;
    case SHT1x_TEMP:
      error |= SHT1x_WriteByte(SHT1x_T_MEASUREMENT);
      break;
  }

  SDA_IN();
  for(i = 0;i < 65535; i++)
  {
    halMcuWaitUs(4);
    if(!IIC_SDA) break;
  }
  if(IIC_SDA) error |= SHT1x_TIME_OUT_ERROR;


  *p_value = SHT1x_ReadByte(1);
  *p_value <<= 8;
  *p_value |= SHT1x_ReadByte(1);
  *p_checksum = SHT1x_ReadByte(0);

  return error;
}


/*************************************************************************************************
* 函数 : SHT1x_ReadStateRegister() => 读取状态寄存器
* 参数 : *stateRegister 状态寄存器数据
* 返回 ：错误信息
*************************************************************************************************/
u8 SHT1x_ReadStateRegister(u8 *stateRegister)
{
  u8 error = 0;
  //u8 checksum = 0;

  SHT1x_TransStart();
  error |= SHT1x_WriteByte(SHT1x_USER_REG_R);
  *stateRegister = SHT1x_ReadByte(0);
  //checksum = SHT1x_ReadByte(0);

  return error;
}



/*************************************************************************************************
* 函数 : SHT1x_ReadTempResult() => 读取真实温度
* 参数 : p_value - 温度，单位 0.01°C，
*************************************************************************************************/
u8 SHT1x_ReadTempResult(u16 *p_value)
{
  u8 error = 0;
  u16 T;
  u8 checksum;
  
  error |= SHT1x_ReadResult(&T,&checksum,SHT1x_TEMP);
  *p_value = -3960 + (int)T;
  
  
  return error;
}


/*************************************************************************************************
* 函数 : SHT1x_ReadRhResult() => 读取真实湿度
* 参数 : p_value - 温度，单位 0.01%HR，
*************************************************************************************************/
u8 SHT1x_ReadRhResult(u16 *p_value)
{
  u8 error = 0;
  u16 RH;
  u8 checksum;
  
  error |= SHT1x_ReadResult(&RH,&checksum,SHT1x_HUMIDITY); 
  *p_value = (uint16)(-204.68f + 3.67f * (float)RH - 0.00015955f * (float)RH * (float)RH);
  
  return error;
}
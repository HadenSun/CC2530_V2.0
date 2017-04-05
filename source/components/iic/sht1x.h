/***********************************************************************************
  Filename:     sht1x.h

  Description:  温湿度传感器头文件

***********************************************************************************/

#ifndef __SHT1X_H
#define __SHT1X_H
/*********************************************头文件包含******************************************/
#include "iic.h"
#include "delay.h"


/************************************************常量定义****************************************/
typedef enum{
	SHT1x_HUMIDITY,
	SHT1x_TEMP
}etSHT1xMeasureType;

typedef enum{
	SHT1x_T_MEASUREMENT 		= 0x03, // 温度测量
	SHT1x_RH_MEASUREMENT		= 0x05, // 湿度测量
	SHT1x_USER_REG_W 			= 0x06, // command writing user register
	SHT1x_USER_REG_R 			= 0x07, // command reading user register
	SHT1x_SOFT_RESET 			= 0x1E 	// command soft reset
}etSHT1xCommand;

typedef enum{
	SHT1x_ACK_ERROR 	= 0x01,
	SHT1x_TIME_OUT_ERROR 	= 0x02,
	SHT1x_CHECKSUM_ERROR 	= 0x04,
	SHT1x_BATTERY_ALERT 	= 0x08
}etSHT1xError;

typedef union
{ 
  u16 i;
  float   f;
}value;

typedef enum{
	SHT1x_I2C_ADR_W = 128, 	// sensor I2C address + write bit
	SHT1x_I2C_ADR_R = 129 	// sensor I2C address + read bit
}etI1cHeader;


/************************************************函数声明****************************************/
void SHT1x_TransStart();        //开始采集
void SHT1x_ReConnect(void);     //重新连接
u8 SHT1x_WriteByte(u8 value);   //写字节数据
u8 SHT1x_ReadByte(u8 ack);      //读取字节数据
u8 SHT1x_ReadResult(u16 *p_value, u8 *p_checksum, u8 mode);    //采集温湿度信息
u8 SHT1x_ReadStateRegister(u8 *stateRegister);                 //读取状态寄存器信息
#endif


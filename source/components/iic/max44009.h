/***********************************************************************************
  Filename:     MAX44009.h

  Description:  光照传感器头文件

***********************************************************************************/
#ifndef __MAX44009_H
#define __MAX44009_H

/*********************************************头文件包含******************************************/
#include "iic.h"
#include "delay.h"

/************************************************常量定义****************************************/
typedef enum{
	MAX_I2C_ADR_W = 0x96, 	// sensor I2C address + write bit
	MAX_I2C_ADR_R = 0x97 	// sensor I2C address + read bit
}MAX_etI2cHeader;

typedef enum{
	INTER_STAT 		= 0x00, // 中断状态寄存器，R
	INTER_EN		= 0x01, // 中断使能寄存器，R/W
	CONFIG     	        = 0x02, // 配置寄存器
	LUX_H 	                = 0x03, // 测量结果高字节
	LUX_L			= 0x04, // 测量结果低字节
	UPPER_THRESHOLD		= 0x05, // 测量上限
	LOWER_THRESHOLD		= 0x06 	// 测量下限
}MAX_Command;


/************************************************函数声明****************************************/

u8 MAX_ReadOriginalData(u8 *RST_H,u8 *RST_L);     //读取测量数据
u8 MAX_ReadLightData(u8 *RST_H,u8 *RST_L);        //转成实际强度

#endif
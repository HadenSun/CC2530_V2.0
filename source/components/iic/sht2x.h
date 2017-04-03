/***********************************************************************************
  Filename:     sht2x.

  Description:  温湿度传感器头文件

***********************************************************************************/

#ifndef __SHT2X_H
#define __SHT2X_H
/*********************************************头文件包含******************************************/
#include "iic.h"
#include "delay.h"


/************************************************常量定义****************************************/
typedef enum{
	TRIG_T_MEASUREMENT_HM 		= 0xE3, // command trig. temp meas. hold master
	TRIG_RH_MEASUREMENT_HM		= 0xE5, // command trig. humidity meas. hold master
	TRIG_T_MEASUREMENT_POLL 	= 0xF3, // command trig. temp meas. no hold master
	TRIG_RH_MEASUREMENT_POLL 	= 0xF5, // command trig. humidity meas. no hold master
	USER_REG_W 								= 0xE6, // command writing user register
	USER_REG_R 								= 0xE7, // command reading user register
	SOFT_RESET 								= 0xFE 	// command soft reset
}etSHT2xCommand;

typedef enum {
	SHT2x_RES_12_14BIT 	= 0x00, // RH=12bit, T=14bit
	SHT2x_RES_8_12BIT 	= 0x01, // RH= 8bit, T=12bit
	SHT2x_RES_10_13BIT 	= 0x80, // RH=10bit, T=13bit
	SHT2x_RES_11_11BIT 	= 0x81, // RH=11bit, T=11bit
	SHT2x_RES_MASK 			= 0x81 	// Mask for res. bits (7,0) in user reg.
} etSHT2xResolution;
typedef enum {
	SHT2x_EOB_ON 		= 0x40, // end of battery
	SHT2x_EOB_MASK 	= 0x40, // Mask for EOB bit(6) in user reg.
} etSHT2xEob;
typedef enum {
	SHT2x_HEATER_ON 	= 0x04, // heater on
	SHT2x_HEATER_OFF 	= 0x00, // heater off
	SHT2x_HEATER_MASK = 0x04, // Mask for Heater bit(2) in user reg.
} etSHT2xHeater;
// measurement signal selection
typedef enum{
	HUMIDITY,
	TEMP
}etSHT2xMeasureType;
typedef enum{
	I2C_ADR_W = 128, 	// sensor I2C address + write bit
	I2C_ADR_R = 129 	// sensor I2C address + read bit
}etI2cHeader;


typedef enum{
	ACK_ERROR 			= 0x01,
	TIME_OUT_ERROR 	= 0x02,
	CHECKSUM_ERROR 	= 0x04,
	BATTERY_ALERT 			= 0x08
}etError;

typedef union {
	u16  ul16; // element specifier for accessing whole u16
	vs16 il16; // element specifier for accessing whole i16
	struct {
		#ifdef LITTLE_ENDIAN // Byte-order is little endian
		u8t u8L; // element specifier for accessing low u8
		u8t u8H; // element specifier for accessing high u8
		#else // Byte-order is big endian
		u8 u8H; // element specifier for accessing low u8
		u8 u8L; // element specifier for accessing high u8
		#endif
	}s16; // element spec. for acc. struct with low or high u8
} nt16;



/************************************************函数声明****************************************/
u8 SHT2x_SoftReset(void);                                                         //软重启
u8 SHT2x_GetSerialNumber(u8 u8SerialNumber[]);                                    //读取设备ID
u8 SHT2x_ReadUserRegister(u8 *pRegisterValue);                                    //读取寄存器
u8 SHT2x_WriteUserRegister(u8 *pRegisterValue);                                   //写入寄存器
u8 SHT2x_CheckCrc(u8 data[], u8 nbrOfBytes, u8 checksum);                         //校验
u8 SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand);     //读取温湿度
float SHT2x_CalcTemperatureC(u16 u16sT);                                          //换算为真实温度
float SHT2x_CalcRH(u16 u16sRH);                                                   //换算为真实湿度


#endif


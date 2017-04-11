/***********************************************************************************
  Filename:     bmp280.h

  Description:  大气压强传感器

***********************************************************************************/
#ifndef __BMP280_H
#define __BMP280_H
/*********************************************头文件包含******************************************/
#include "iic.h"
#include "delay.h"


/************************************************常量定义****************************************/
typedef enum{
  BMP_TEMP_XLSB   = 0xFC,
  BMP_TEMP_LSB    = 0xFB,
  BMP_TEMP_MSB    = 0xFA,
  BMP_PRESS_XLSB  = 0xF9,
  BMP_PRESS_LSB   = 0xF8,
  BMP_PRESS_MSB   = 0xF7,
  BMP_CONFIG      = 0xF5,
  BMP_CTRL_MEAS   = 0xF4,
  BMP_STATUES     = 0xF3,
  BMP_RESET       = 0xE0,
  BMP_ID          = 0xD0,
  BMP_DIG_T1      = 0x88,
  BMP_DIG_T2      = 0x8A,
  BMP_DIG_T3      = 0x8C,
  BMP_DIG_P1      = 0x8E,
  BMP_DIG_P2      = 0x90,
  BMP_DIG_P3      = 0x92,
  BMP_DIG_P4      = 0x94,
  BMP_DIG_P5      = 0x96,
  BMP_DIG_P6      = 0x98,
  BMP_DIG_P7      = 0x9A,
  BMP_DIG_P8      = 0x9C,
  BMP_DIG_P9      = 0x9E
}bmpRegisters;        //寄存器地址


typedef enum{
  BMP_ADD_W = 0xEC,
  BMP_ADD_R = 0xED
}bmpI2CAdd;              //I2C从设备地址，SDO接地


typedef enum{
  BMP_PRESS,
  BMP_TEMP
}bmpMeasureType;


typedef union{
  unsigned short t1;
  short t2;
  short t3;
  unsigned short p1;
  short p2;
  short p3;
  short p4;
  short p5;
  short p6;
  short p7;
  short p8;
  short p9;
  
}bmp_dig;


/************************************************函数声明****************************************/
u8 BMP280_ReadId(u8 *id);       //读取设备ID
u8 BMP280_Reset();              //软重启
u8 BMP280_TransStart();         //测量开始
u8 BMP280_Init();               //初始化
u8 BMP280_ReadResult(u16 *data,u8 *xdata,bmpMeasureType mode);    //读取测量数据
u8 BMP280_ReadRegister(u8 registerAdd,u8 *data);

#endif
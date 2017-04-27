/***********************************************************************************
  Filename:     delay.h

  Description:  延时函数头文件；定义一些类型缩写

***********************************************************************************/


#ifndef _DELAY_H
#define _DELAY_H

/************************************************常量定义****************************************/
#define uint unsigned int
#define uchar unsigned char
#define u8 unsigned char
#define u16 unsigned int
typedef signed short int vs16;


/************************************************结构定义****************************************/
struct sensor_data{
  unsigned short AirPressure[8];
  unsigned short Illumination[8];
  unsigned short Temperature[8];
  unsigned short Humidity[8];
  unsigned char error[8];
}WireSensorData;


unsigned short co2Concentration[2];



/************************************************函数声明****************************************/
//延时函数
void delay_us(uint n);    //n*3 us
void delay_ms(uint n);    //n*1 ms

#endif

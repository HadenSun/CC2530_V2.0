/***********************************************************************************
  Filename:     delay.h

  Description:  ��ʱ����ͷ�ļ�������һЩ������д

***********************************************************************************/


#ifndef _DELAY_H
#define _DELAY_H

/************************************************��������****************************************/
#define uint unsigned int
#define uchar unsigned char
#define u8 unsigned char
#define u16 unsigned int
typedef signed short int vs16;


/************************************************�ṹ����****************************************/
struct sensor_data{
  unsigned short AirPressure[8];
  unsigned short Illumination[8];
  unsigned short Temperature[8];
  unsigned short Humidity[8];
  unsigned char error[8];
}WireSensorData;


unsigned short co2Concentration[2];



/************************************************��������****************************************/
//��ʱ����
void delay_us(uint n);    //n*3 us
void delay_ms(uint n);    //n*1 ms

#endif

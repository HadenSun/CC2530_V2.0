/***********************************************************************************
  Filename:     uart.c

  Description:  串口代码

***********************************************************************************/



/*********************************************头文件包含******************************************/
#include "uart.h"
#include "hal_types.h"
#include <ioCC2530.h>


/*************************************************************************************************
* 函数 ：Uart1_Init() => Uart1初始化
* 说明 ：使用Uart1默认引脚，P0.4-TX，P0.5-RX
* 参数 ：baud - 波特率
*      ：isInt - 是否使用接收中断
*************************************************************************************************/
void Uart1_Init(u8 baud ,u8 isInt)
{

    PERCFG = 0x00;                //UART1使用默认引脚 P0.4 P0.5
    P0SEL = 0x30;                 //P0.4 P0.5复用
    P2DIR |= 0x40;                //UART1优先

    U1CSR |= 0x80;                //选择UART模式
    switch(baud)
    {
      case UART_BAUD_115200:
        U1GCR |= 11;
        U1BAUD |= 216;                //波特率115200
        break;
      case UART_BAUD_38400:
        U1GCR |= 10;
        U1BAUD |= 59;
        break;
    }
    UTX1IF = 0;                   //清除中断标志
    U1CSR |= 0x40;                //UART接收使能
    if(isInt)
      IEN0 |= 0x88;                 //总中断使能，UART1 RX中断使能
}

/*************************************************************************************************
* 函数 ：Uart1_SendString(char *Date,int len) => UART1发送字符串
*************************************************************************************************/
void Uart1_SendString(char *Date,int len)
{
    int j;
    for(j = 0; j < len;j++)
    {
        U1DBUF = *Date++;
        while(!UTX1IF);
        UTX1IF = 0;
    }
}

/*************************************************************************************************
* 函数 ：Uart1_SendByte(uchar n) => UART1发送字节
*************************************************************************************************/
void Uart1_SendByte(uchar n)
{
  U1DBUF = n;
  while(!UTX1IF);
  UTX1IF = 0;
}


/*************************************************************************************************
* 函数 ：Uart1_SendData(void) => UART1发送字节
*************************************************************************************************/
void Uart1_SendData()
{
  u8 i = 0;
  uint16 temp = 0;
  uint32 press = 0;

  Uart1_SendByte(0x01);     //起始位
  Uart1_SendByte(0X02);     //起始位
  Uart1_SendByte(0x01);     //包含CO2
  for(i = 0;i < 8;i++)
  {
    Uart1_SendByte(i+1);    //地址
    Uart1_SendByte(WireSensorData.error[i]);     //错误信息
    temp = WireSensorData.Temperature[i];
    Uart1_SendByte(temp/1000 + '0');    //温度十位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //温度个位
    Uart1_SendByte('.');          //小数点
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //一位小数

    temp = WireSensorData.Humidity[i];
    Uart1_SendByte(temp/1000 + '0');    //湿度十位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //湿度个位
    Uart1_SendByte('.');          //小数点
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //一位小数

    if(WireSensorData.AirPressure[i])
      press = (uint32)WireSensorData.AirPressure[i] | 0x10000;
    else
      press = 0;
    Uart1_SendByte(press/100000 + '0');    //气压千位
    press = press % 100000;
    Uart1_SendByte(press / 10000 + '0');   //气压百位
    press = press % 10000;
    Uart1_SendByte(press / 1000 + '0');    //气压十位
    press = press % 1000;
    Uart1_SendByte(press / 100 + '0');     //气压个位，单位hPa

    temp = WireSensorData.Illumination[i];
    Uart1_SendByte(temp/1000 + '0');    //光照千位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //光照百位
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //光照十位
    temp = temp % 10;
    Uart1_SendByte(temp + '0');         //光照个位
  }

  temp = co2Concentration[0];
  Uart1_SendByte(0x09);
  Uart1_SendByte(4);
  Uart1_SendByte(temp/1000 + '0');    //CO2千位
  temp = temp % 1000;
  Uart1_SendByte(temp/100 + '0');     //CO2百位
  temp = temp % 100;
  Uart1_SendByte(temp / 10 + '0');    //CO2十位
  temp = temp % 10;
  Uart1_SendByte(temp + '0');         //CO2个位

  temp = co2Concentration[1];
  Uart1_SendByte(0x0A);           //CO2 地址
  Uart1_SendByte(4);              //长读
  Uart1_SendByte(temp/1000 + '0');    //CO2千位
  temp = temp % 1000;
  Uart1_SendByte(temp/100 + '0');     //CO2百位
  temp = temp % 100;
  Uart1_SendByte(temp / 10 + '0');    //CO2十位
  temp = temp % 10;
  Uart1_SendByte(temp + '0');         //CO2个位
}

void Uart1_SendData_Test(u8 i)
{
    u8 temp;
    uint32 press;

    Uart1_SendByte(i+1+'0');    //地址
    Uart1_SendByte(WireSensorData.error[i]);
    temp = WireSensorData.Temperature[i];
    Uart1_SendByte(temp/1000 + '0');    //温度十位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //温度个位
    Uart1_SendByte('.');          //小数点
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //一位小数

    temp = WireSensorData.Humidity[i];
    Uart1_SendByte(temp/1000 + '0');    //湿度十位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //湿度个位
    Uart1_SendByte('.');          //小数点
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //一位小数

    if(WireSensorData.AirPressure[i])
      press = (uint32)WireSensorData.AirPressure[i] | 0x10000;
    Uart1_SendByte(press/100000 + '0');    //气压千位
    press = press % 100000;
    Uart1_SendByte(press / 10000 + '0');   //气压百位
    press = press % 10000;
    Uart1_SendByte(press / 1000 + '0');    //气压十位
    press = press % 1000;
    Uart1_SendByte(press / 100 + '0');     //气压个位，单位hPa

    temp = WireSensorData.Illumination[i];
    Uart1_SendByte(temp/1000 + '0');    //光照千位
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //光照百位
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //光照十位
    temp = temp % 10;
    Uart1_SendByte(temp + '0');         //光照个位
}

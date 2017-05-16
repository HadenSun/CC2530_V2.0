/***********************************************************************************
  Filename:     uart.c

  Description:  串口代码

***********************************************************************************/



/*********************************************头文件包含******************************************/
#include "uart.h"
#include "hal_types.h"
#include <ioCC2530.h>


static uint8 aucCRCHi[] = {
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
     0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
     0x00, 0xC1, 0x81, 0x40};
static uint8 aucCRCLo[] = {
     0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
     0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
     0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
     0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
     0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
     0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
     0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
     0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
     0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
     0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
     0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
     0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
     0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
     0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
     0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
     0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
     0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
     0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
     0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
     0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
     0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
     0x41, 0x81, 0x80, 0x40
 };
u8 UartTxData[161] = {0};

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
void Uart1_SendString(u8 *Date,int len)
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
void Crc16(uint8* pucFrame, uint8 usLen,uint8 *crc_1,uint8 *crc_2)
{
    uint8 i = 0;
    uint8 ucCRCHi = 0xFF;
    uint8 ucCRCLo = 0xFF;
    uint16 iIndex = 0x0000;

    while (usLen-- > 0)
    {
        iIndex = (uint16)(ucCRCLo ^ pucFrame[i++]);
        ucCRCLo = (uint8)(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    
    *crc_1 = ucCRCLo;
    *crc_2 = ucCRCHi;

}


/*************************************************************************************************
* 函数 ：Uart1_SendData(void) => UART1发送字节
*************************************************************************************************/
void Uart1_SendData()
{
  u8 i = 0;
  uint16 temp = 0;
  uint32 press = 0;
  u8 crc_1;
  u8 crc_2;

  UartTxData[0] = 0x01;   //起始位
  UartTxData[1] = 0x02;
  UartTxData[2] = 0x01;   //包含CO2
  
  for(i = 0;i < 8; i++)
  {
    UartTxData[i * 18 + 3] = i+1;         //地址
    UartTxData[i * 18 + 4] = WireSensorData.error[i];   //错误信息
    temp = WireSensorData.Temperature[i];
    UartTxData[i * 18 + 5] = temp/1000 + '0';     //温度十位
    temp = temp % 1000;
    UartTxData[i * 18 + 6] = temp/100 + '0';      //温度个位
    UartTxData[i * 18 + 7] = '.';                 //小数点
    temp = temp % 100;
    UartTxData[i * 18 + 8] = temp / 10 + '0';     //一位小数

    temp = WireSensorData.Humidity[i];
    UartTxData[i * 18 + 9] = temp/1000 + '0';     //湿度十位
    temp = temp % 1000;
    UartTxData[i * 18 + 10] = temp/100 + '0';     //湿度个位
    UartTxData[i * 18 + 11] = '.';                //小数点
    temp = temp % 100;
    UartTxData[i * 18 + 12] = temp / 10 + '0';    //一位小数

    if(WireSensorData.AirPressure[i])
      press = (uint32)WireSensorData.AirPressure[i] | 0x10000;
    else
      press = 0;
    UartTxData[i * 18 + 13] = press/100000 + '0';    //气压千位
    press = press % 100000;
    UartTxData[i * 18 + 14] = press / 10000 + '0';   //气压百位
    press = press % 10000;
    UartTxData[i * 18 + 15] = press / 1000 + '0';    //气压十位
    press = press % 1000;
    UartTxData[i * 18 + 16] = press / 100 + '0';     //气压个位，单位hPa

    temp = WireSensorData.Illumination[i];
    UartTxData[i * 18 + 17] = temp/1000 + '0';    //光照千位
    temp = temp % 1000;
    UartTxData[i * 18 + 18] = temp/100 + '0';     //光照百位
    temp = temp % 100;
    UartTxData[i * 18 + 19] = temp / 10 + '0';    //光照十位
    temp = temp % 10;
    UartTxData[i * 18 + 20] = temp + '0';         //光照个位
  }
  
  temp = co2Concentration[0];
  UartTxData[147] = 0x09;               //CO2地址
  UartTxData[148] = 0;                  //error
  UartTxData[149] = temp/1000 + '0';    //CO2千位
  temp = temp % 1000;
  UartTxData[150] = temp/100 + '0';     //CO2百位
  temp = temp % 100;
  UartTxData[151] = temp / 10 + '0';    //CO2十位
  temp = temp % 10;
  UartTxData[152] = temp + '0';         //CO2个位

  temp = co2Concentration[1];
  UartTxData[153] = 0x0A;               //CO2 地址
  UartTxData[154] = 0;                  //error
  UartTxData[155] = temp/1000 + '0';    //CO2千位
  temp = temp % 1000;
  UartTxData[156] = temp/100 + '0';     //CO2百位
  temp = temp % 100;
  UartTxData[157] = temp / 10 + '0';    //CO2十位
  temp = temp % 10;
  UartTxData[158] = temp + '0';         //CO2个位
  
  Crc16(UartTxData, 159,&crc_1,&crc_2); //CRC校验位
  UartTxData[159] = crc_1;
  UartTxData[160] = crc_2;
  
  Uart1_SendString(UartTxData,161);     //通过串口发送数据
  
}

/*************************************************************************************************
* 函数 ：Uart1_SendData_Test(void) => UART1发送测试数据
*************************************************************************************************/
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


/*************************************************************************************************
* 函数 ：Uart1_SendData_Test(void) => UART1发送测试数据
* 说明 ：串口数据格式为：起始位（0xA5）+命令+数据+校验位
*        校验位为前面位数之和，包括起始位，如果和校验为A5则+1
*************************************************************************************************/
u8 Uart_CheckSum(u8 *pData)
{
  u8 checksum = 0;
  checksum += *pData++;
  checksum += *pData++;
  checksum += *pData++;
  if(checksum == 0xA5)
    checksum++;
  if(checksum == *pData)
    return 0;

  return 1;
}
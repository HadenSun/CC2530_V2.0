/***********************************************************************************
  Filename:     uart.c

  Description:  ���ڴ���

***********************************************************************************/



/*********************************************ͷ�ļ�����******************************************/
#include "uart.h"
#include "hal_types.h"
#include <ioCC2530.h>


/*************************************************************************************************
* ���� ��Uart1_Init() => Uart1��ʼ��
* ˵�� ��ʹ��Uart1Ĭ�����ţ�P0.4-TX��P0.5-RX
* ���� ��baud - ������
*      ��isInt - �Ƿ�ʹ�ý����ж�
*************************************************************************************************/
void Uart1_Init(u8 baud ,u8 isInt)
{

    PERCFG = 0x00;                //UART1ʹ��Ĭ������ P0.4 P0.5
    P0SEL = 0x30;                 //P0.4 P0.5����
    P2DIR |= 0x40;                //UART1����

    U1CSR |= 0x80;                //ѡ��UARTģʽ
    switch(baud)
    {
      case UART_BAUD_115200:
        U1GCR |= 11;
        U1BAUD |= 216;                //������115200
        break;
      case UART_BAUD_38400:
        U1GCR |= 10;
        U1BAUD |= 59;
        break;
    }
    UTX1IF = 0;                   //����жϱ�־
    U1CSR |= 0x40;                //UART����ʹ��
    if(isInt)
      IEN0 |= 0x88;                 //���ж�ʹ�ܣ�UART1 RX�ж�ʹ��
}

/*************************************************************************************************
* ���� ��Uart1_SendString(char *Date,int len) => UART1�����ַ���
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
* ���� ��Uart1_SendByte(uchar n) => UART1�����ֽ�
*************************************************************************************************/
void Uart1_SendByte(uchar n)
{
  U1DBUF = n;
  while(!UTX1IF);
  UTX1IF = 0;
}


/*************************************************************************************************
* ���� ��Uart1_SendData(void) => UART1�����ֽ�
*************************************************************************************************/
void Uart1_SendData()
{
  u8 i = 0;
  uint16 temp = 0;
  uint32 press = 0;

  Uart1_SendByte(0x01);     //��ʼλ
  Uart1_SendByte(0X02);     //��ʼλ
  Uart1_SendByte(0x01);     //����CO2
  for(i = 0;i < 8;i++)
  {
    Uart1_SendByte(i+1);    //��ַ
    Uart1_SendByte(WireSensorData.error[i]);     //������Ϣ
    temp = WireSensorData.Temperature[i];
    Uart1_SendByte(temp/1000 + '0');    //�¶�ʮλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //�¶ȸ�λ
    Uart1_SendByte('.');          //С����
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //һλС��

    temp = WireSensorData.Humidity[i];
    Uart1_SendByte(temp/1000 + '0');    //ʪ��ʮλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //ʪ�ȸ�λ
    Uart1_SendByte('.');          //С����
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //һλС��

    if(WireSensorData.AirPressure[i])
      press = (uint32)WireSensorData.AirPressure[i] | 0x10000;
    else
      press = 0;
    Uart1_SendByte(press/100000 + '0');    //��ѹǧλ
    press = press % 100000;
    Uart1_SendByte(press / 10000 + '0');   //��ѹ��λ
    press = press % 10000;
    Uart1_SendByte(press / 1000 + '0');    //��ѹʮλ
    press = press % 1000;
    Uart1_SendByte(press / 100 + '0');     //��ѹ��λ����λhPa

    temp = WireSensorData.Illumination[i];
    Uart1_SendByte(temp/1000 + '0');    //����ǧλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //���հ�λ
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //����ʮλ
    temp = temp % 10;
    Uart1_SendByte(temp + '0');         //���ո�λ
  }

  temp = co2Concentration[0];
  Uart1_SendByte(0x09);
  Uart1_SendByte(4);
  Uart1_SendByte(temp/1000 + '0');    //CO2ǧλ
  temp = temp % 1000;
  Uart1_SendByte(temp/100 + '0');     //CO2��λ
  temp = temp % 100;
  Uart1_SendByte(temp / 10 + '0');    //CO2ʮλ
  temp = temp % 10;
  Uart1_SendByte(temp + '0');         //CO2��λ

  temp = co2Concentration[1];
  Uart1_SendByte(0x0A);           //CO2 ��ַ
  Uart1_SendByte(4);              //����
  Uart1_SendByte(temp/1000 + '0');    //CO2ǧλ
  temp = temp % 1000;
  Uart1_SendByte(temp/100 + '0');     //CO2��λ
  temp = temp % 100;
  Uart1_SendByte(temp / 10 + '0');    //CO2ʮλ
  temp = temp % 10;
  Uart1_SendByte(temp + '0');         //CO2��λ
}

void Uart1_SendData_Test(u8 i)
{
    u8 temp;
    uint32 press;

    Uart1_SendByte(i+1+'0');    //��ַ
    Uart1_SendByte(WireSensorData.error[i]);
    temp = WireSensorData.Temperature[i];
    Uart1_SendByte(temp/1000 + '0');    //�¶�ʮλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //�¶ȸ�λ
    Uart1_SendByte('.');          //С����
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //һλС��

    temp = WireSensorData.Humidity[i];
    Uart1_SendByte(temp/1000 + '0');    //ʪ��ʮλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //ʪ�ȸ�λ
    Uart1_SendByte('.');          //С����
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //һλС��

    if(WireSensorData.AirPressure[i])
      press = (uint32)WireSensorData.AirPressure[i] | 0x10000;
    Uart1_SendByte(press/100000 + '0');    //��ѹǧλ
    press = press % 100000;
    Uart1_SendByte(press / 10000 + '0');   //��ѹ��λ
    press = press % 10000;
    Uart1_SendByte(press / 1000 + '0');    //��ѹʮλ
    press = press % 1000;
    Uart1_SendByte(press / 100 + '0');     //��ѹ��λ����λhPa

    temp = WireSensorData.Illumination[i];
    Uart1_SendByte(temp/1000 + '0');    //����ǧλ
    temp = temp % 1000;
    Uart1_SendByte(temp/100 + '0');     //���հ�λ
    temp = temp % 100;
    Uart1_SendByte(temp / 10 + '0');    //����ʮλ
    temp = temp % 10;
    Uart1_SendByte(temp + '0');         //���ո�λ
}

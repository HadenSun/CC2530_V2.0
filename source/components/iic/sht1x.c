/***********************************************************************************
  Filename:     sht1x.c

  Description:  ��ʪ�ȴ�����

***********************************************************************************/

/*********************************************ͷ�ļ�����******************************************/
#include "sht1x.h"
#include "delay.h"

#include <hal_mcu.h>


/************************************************��������****************************************/
const u16 SHT1x_POLYNOMIAL = 0x131;		//P(x)=x^8+x^5+x^4+1=100110001




/*************************************************************************************************
* ���� ��   SHT1x_TransStart() => ��ʼ�ɼ�
* ���� �� generates a transmission start
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
* ���� ��SHT1x_ReConnect() => ��������
* ˵�� ������ά��9�� SCL ���ڵ� SDA �ߵ�ƽ
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
* ���� : SHT1x_WriteByte(u8 value) => д�ֽ�����
* ���� : value - д������
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
* ���� �� SHT1x_ReadByte(u8 ack) => ��ȡ����
* ���� �� ack=1 Ӧ��ack=0 ��Ӧ��
* ���� �� ��ȡ��������
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
* ���� : SHT1x_ReadResult() => �ɼ���ʪ����Ϣ
* ���� : * p_value �ɼ���������
        * p_checksum У����Ϣ
        mode �¶ȣ�ʪ��
* ���� ���ɼ�״̬
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
    default:
      break;
  }

  SDA_IN();
  for(i = 0;i < 65535; i++)
  {
    halMcuWaitUs(4);
    if(!IIC_SDA) break;
  }
  if(IIC_SDA) error |= SHT1x_ACK_ERROR;

  SDA_OUT();

  *p_value = SHT1x_ReadByte(1);
  *p_value <<= 8;
  *p_value |= SHT1x_ReadByte(1);
  *p_checksum = SHT1x_ReadByte(0);

  return error;
}


/*************************************************************************************************
* ���� : SHT1x_ReadStateRegister() => ��ȡ״̬�Ĵ���
* ���� : *stateRegister ״̬�Ĵ�������
* ���� ��������Ϣ
*************************************************************************************************/
u8 SHT1x_ReadStateRegister(u8 *stateRegister)
{
  u8 error = 0;
  u8 checksum = 0;

  SHT1x_TransStart();
  error |= SHT1x_WriteByte(SHT1x_USER_REG_R);
  *stateRegister = SHT1x_ReadByte(1);
  checksum = SHT1x_ReadByte(0);

  return error;
}

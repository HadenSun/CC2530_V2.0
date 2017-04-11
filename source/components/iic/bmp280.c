/***********************************************************************************
  Filename:     bmp280.c

  Description:  ����ѹǿ������

***********************************************************************************/

/*********************************************ͷ�ļ�����******************************************/
#include "bmp280.h"


bmp_dig Bmp_Dig;


/*************************************************************************************************
* ���� ��   BMP280_WriteRegister(u8 registerAddress,u8 data) => д��Ĵ���
* ���� ��   registerAddress - �Ĵ�����ַ
            data - д��Ĵ�������
*************************************************************************************************/
u8 BMP280_WriteRegister(u8 registerAddress,u8 data)
{
  u8 error = 0;
  IIC_Start();
  IIC_Send_Byte(BMP_ADD_W);
  error |= IIC_Wait_Ack();
  IIC_Send_Byte(registerAddress);
  error |= IIC_Wait_Ack();
  IIC_Send_Byte(data);
  error |= IIC_Wait_Ack();
  
  IIC_Stop();
  
  return error;
}


/*************************************************************************************************
* ���� ��   BMP280_ReadRegister(u8 registerAdd,u8 *data) => ��ȡ�Ĵ�������
* ���� ��   registerAddress - �Ĵ�����ַ
            data - ��ȡ�Ĵ�������
*************************************************************************************************/
u8 BMP280_ReadRegister(u8 registerAdd,u8 *data)
{
  u8 error = 0;
  
  IIC_Start();
  IIC_Send_Byte(BMP_ADD_W);
  error |= IIC_Wait_Ack();
  IIC_Send_Byte(registerAdd);
  error |= IIC_Wait_Ack();
  
  IIC_Start();
  IIC_Send_Byte(BMP_ADD_R);
  error |= IIC_Wait_Ack();
  *data = IIC_Read_Byte(0);
  
  IIC_Stop();
  
  return error;
}







/*************************************************************************************************
* ���� ��   BMP280_ReadId() => ��ȡ�豸ID
* ���� ��   *id - ������ID
*************************************************************************************************/
u8 BMP280_ReadId(u8 *id)
{
  u8 error = 0;
  
  error |= BMP280_ReadRegister(BMP_ID,id);
  
  return error;
}


/*************************************************************************************************
* ���� ��   BMP280_Reset() => ������

*************************************************************************************************/
u8 BMP280_Reset()
{
  u8 error = 0;
  
  error |= BMP280_WriteRegister(BMP_RESET,0xB6);      //������д��0xB6��д������������Ч
  
  return error;
}

/*************************************************************************************************
* ���� ��   BMP280_Init() => ��ʼ��

*************************************************************************************************/
u8 BMP280_Init()
{
  u8 error = 0;
  u8 temp = 0;
  u16 t = 0;
  
  error |= BMP280_WriteRegister(BMP_CONFIG,0x00);
  error |= BMP280_WriteRegister(BMP_CTRL_MEAS,0x25);
  
  error |= BMP280_ReadRegister(BMP_DIG_T1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T1+1,&temp);
  t |= temp;
  Bmp_Dig.t1 = (unsigned short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_T2,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T2+1,&temp);
  t |= temp;
  Bmp_Dig.t2 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_T3,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T3+1,&temp);
  t |= temp;
  Bmp_Dig.t3 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P1+1,&temp);
  t |= temp;
  Bmp_Dig.p1 = (unsigned short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P2,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P2+1,&temp);
  t |= temp;
  Bmp_Dig.p2 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P3,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P3+1,&temp);
  t |= temp;
  Bmp_Dig.p3 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P4,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P4+1,&temp);
  t |= temp;
  Bmp_Dig.p4 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P5,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P5+1,&temp);
  t |= temp;
  Bmp_Dig.p5 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P6,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P6+1,&temp);
  t |= temp;
  Bmp_Dig.p6 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P7,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P7+1,&temp);
  t |= temp;
  Bmp_Dig.p7 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P8,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P8+1,&temp);
  t |= temp;
  Bmp_Dig.p8 = (short)t;
  
  error |= BMP280_ReadRegister(BMP_DIG_P9,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P9+1,&temp);
  t |= temp;
  Bmp_Dig.p9 = (short)t;
  
  return error;
}

/*************************************************************************************************
* ���� ��   BMP280_TransStart() => һ�β�����ʼ

*************************************************************************************************/
u8 BMP280_TransStart()
{
  u8 error = 0;
  
  error |= BMP280_WriteRegister(BMP_CTRL_MEAS,0x25);
  delay_ms(100);
  
  return error;
}


/*************************************************************************************************
* ���� ��   BMP280_ReadResult() => ��ȡ�������
* ���� ��*data - �������
*        *xdata - �ֱ���
*        mode - ��ȡ����
*************************************************************************************************/
u8 BMP280_ReadResult(u16 *data,u8 *xdata,bmpMeasureType mode)
{
  u8 error = 0;
  u8 temp = 0;
  
  switch(mode)
  {
    case BMP_PRESS:
      *data = 0;
      error |= BMP280_ReadRegister(BMP_PRESS_MSB,&temp);
      *data = temp << 8;
      error |= BMP280_ReadRegister(BMP_PRESS_LSB,&temp);
      *data |= temp;
      error |= BMP280_ReadRegister(BMP_PRESS_XLSB,xdata);
      break;
    case BMP_TEMP:
      *data = 0;
      error |= BMP280_ReadRegister(BMP_TEMP_MSB,&temp);
      *data = temp << 8;
      error |= BMP280_ReadRegister(BMP_TEMP_LSB,&temp);
      *data |= temp;
      error |= BMP280_ReadRegister(BMP_TEMP_XLSB,xdata);
      break;
  }
  
  return error;
}
/***********************************************************************************
  Filename:     sht2x.c

  Description:  ��ʪ�ȴ�����

***********************************************************************************/

/*********************************************ͷ�ļ�����******************************************/
#include "sht2x.h"
#include "delay.h"


/************************************************��������****************************************/
const u16 POLYNOMIAL = 0x131;		//P(x)=x^8+x^5+x^4+1=100110001




/*************************************************************************************************
* ���� ��   SHT2x_SoftReset() => ������
*************************************************************************************************/
u8 SHT2x_SoftReset()
{
	u8 error = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(SOFT_RESET);
	error |= IIC_Wait_Ack();
	IIC_Stop();

	delay_ms(15000);
	return error;
}

/*************************************************************************************************
* ���� ��SHT2x_GetSerialNumber() => ��ȡ�豸ID
*************************************************************************************************/
u8 SHT2x_GetSerialNumber(u8 u8SerialNumber[])
{
	u8 error = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(0xFA);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(0x0F);
	error |= IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_ADR_R);
	error |= IIC_Wait_Ack();
	u8SerialNumber[5] = IIC_Read_Byte(1);
	IIC_Read_Byte(1);
	u8SerialNumber[4] = IIC_Read_Byte(1);
	IIC_Read_Byte(1);
	u8SerialNumber[3] = IIC_Read_Byte(1);
	IIC_Read_Byte(1);
	u8SerialNumber[2] = IIC_Read_Byte(1);
	IIC_Read_Byte(0);
	IIC_Stop();

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(0xFC);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(0xC9);
	error |= IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_ADR_R);
	error |= IIC_Wait_Ack();
	u8SerialNumber[1] = IIC_Read_Byte(1);
	u8SerialNumber[0] = IIC_Read_Byte(1);
	IIC_Read_Byte(1);
	u8SerialNumber[7] = IIC_Read_Byte(1);
	u8SerialNumber[6] = IIC_Read_Byte(1);
	IIC_Read_Byte(0);
	IIC_Stop();

	return error;
}

/*************************************************************************************************
* ���� : SHT2x_ReadUserRegister() => ��ȡ�Ĵ���                      *
*************************************************************************************************/
u8 SHT2x_ReadUserRegister(u8 *pRegisterValue)
{
	u8 checksum;
	u8 error = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(USER_REG_R);
	error |= IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_ADR_R);
	error |= IIC_Wait_Ack();
	*pRegisterValue = IIC_Read_Byte(1);
	checksum = IIC_Read_Byte(0);
	error |= SHT2x_CheckCrc(pRegisterValue,1,checksum);
	IIC_Stop();
	return error;
}

/*************************************************************************************************
* ���� ��SHT2x_WriteUserRegister() => д��Ĵ���
*************************************************************************************************/
u8 SHT2x_WriteUserRegister(u8 *pRegisterValue)
{
	u8 error = 0;

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(USER_REG_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(*pRegisterValue);
	error |= IIC_Wait_Ack();
	IIC_Stop();

	return error;
}

/*************************************************************************************************
* ���� : SHT2x_CheckCrc() => У��

*************************************************************************************************/
u8 SHT2x_CheckCrc(u8 data[], u8 nbrOfBytes, u8 checksum)
{
	u8 crc = 0,bit;
	u8 byteCtr;

	for(byteCtr = 0; byteCtr < nbrOfBytes; ++ byteCtr)
	{
		crc ^= (data[byteCtr]);
		for(bit = 8;bit > 0; bit-- )
		{
			if(crc & 0x80)
				crc = (crc << 1) ^ POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	if(crc != checksum)
		return CHECKSUM_ERROR;
	else
		return 0;
}

/*************************************************************************************************
* ���� : SHT2x_MeasurePoll() => ��ȡ��ʪ����Ϣ
*************************************************************************************************/
u8 SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
{
	u8 checksum;
	u8 data[2];
	u8 error = 0;
	u16 i=0;

	IIC_Start();
	IIC_Send_Byte(I2C_ADR_W);
	error |= IIC_Wait_Ack();
	switch (eSHT2xMeasureType) {
		case HUMIDITY:
			IIC_Send_Byte(TRIG_RH_MEASUREMENT_POLL);
			error |= IIC_Wait_Ack();
			break;
		case TEMP:
			IIC_Send_Byte(TRIG_T_MEASUREMENT_POLL);
			error |= IIC_Wait_Ack();
			break;
	}
	do {
		IIC_Start();
		delay_ms(10);
		if(i++ >= 20)
			break;

		IIC_Send_Byte(I2C_ADR_R);
		if(!IIC_Wait_Ack())
			break;
	} while(1);

	if(i>=20)
		error |= TIME_OUT_ERROR;

	pMeasurand->s16.u8H = data[0] = IIC_Read_Byte(1);
	pMeasurand->s16.u8L = data[1] = IIC_Read_Byte(1);

	checksum = IIC_Read_Byte(0);

	error |= SHT2x_CheckCrc(data,2,checksum);
	IIC_Stop();

	return error;

}


/*************************************************************************************************
* ���� : SHT2x_CalcTemperatureC() => �����¶�
*************************************************************************************************/
float SHT2x_CalcTemperatureC(u16 u16sT)
{
	float temperatureC;

	u16sT &= ~0X0003;
	temperatureC = -46.85f + 175.72/65536 * (float)u16sT;

	return temperatureC;
}

/*************************************************************************************************
* ���� : SHT2x_CalcRH() => ����ʪ����Ϣ
*************************************************************************************************/
float SHT2x_CalcRH(u16 u16sRH)
{
	float humidityRH;

	u16sRH &= ~0x0003;

	humidityRH = -6.0f + 125.0/65536 * (float)u16sRH;

	return humidityRH;
}
/***********************************************************************************
  Filename:     opt3001.c

  Description:  光照传感器

***********************************************************************************/

/*********************************************头文件包含******************************************/
#include "opt3001.h"
#include "delay.h"
#include "iic.h"




/*************************************************************************************************
* 函数 ：   OPT3001_ReadManufactureerID() => 读取制造商ID
*************************************************************************************************/
u8 OPT3001_ReadManufactureerID(u16 *MID)
{
	u8 error = 0;
	u8 rst = 0;
	
	//选择要读取的制造商ID寄存器
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(OPT3001_REG_ADD_MID);
	error |= IIC_Wait_Ack();
	IIC_Stop();
	delay_us(2);
	
	//读取寄存器内容
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_R);
	error |= IIC_Wait_Ack();
	rst = IIC_Read_Byte(1);
	*MID = (rst << 8);
	rst = IIC_Read_Byte(0);
	*MID |= rst;
	IIC_Stop();
	
	
	return error;
}

/*************************************************************************************************
* 函数 ：OPT3001_ReadDeviceID() => 读取设备ID
*************************************************************************************************/
u8 OPT3001_ReadDeviceID(u16 *DID)
{
	u8 error = 0;
	u8 rst = 0;
	
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(OPT3001_REG_ADD_DID);
	error |= IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(2);
	
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_R);
	error |= IIC_Wait_Ack();
	rst = IIC_Read_Byte(1);
	*DID = (rst << 8);
	rst = IIC_Read_Byte(0);
	*DID |= rst;
	IIC_Stop();
	
	return error;
}

/*************************************************************************************************
* 函数 : OPT3001_WriteRegister() => 寄存器写入                              *
*************************************************************************************************/
u8 OPT3001_WriteRegister(u8 add,u16 reg)
{
	u8 error = 0;
	
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(add);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(reg & 0xFF);
	error |= IIC_Wait_Ack();
	IIC_Stop();
	
	return error;
}

/*************************************************************************************************
* 函数 ：OPT3001_ReadResult() => 读取光照强度转换结果
*************************************************************************************************/
u8 OPT3001_ReadResult(float *rst)
{
	u8 error = 0;
	u8 temp = 0;
	u16 re = 0;
	u8 e = 0;
	
	*rst = 0;
	
	//改变寄存器指针
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(OPT3001_REG_ADD_RST);
	error |= IIC_Wait_Ack();
	IIC_Stop();
	
	delay_us(1);
	
	//读取结果
	IIC_Start();
	IIC_Send_Byte(OPT3001_ADD_R);
	error |= IIC_Wait_Ack();
	temp = IIC_Read_Byte(1);
	e = temp >> 4;
	re = (temp & 0x0F) << 8;
	temp = IIC_Read_Byte(0);
	IIC_Stop();
	
	re |= temp;
	*rst = re * 0.01f * (1 << e);
	
	
	return error;
}



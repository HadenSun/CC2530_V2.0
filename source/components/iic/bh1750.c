/***********************************************************************************
  Filename:     bh1750.c

  Description:  光线强度传感器

***********************************************************************************/

/*********************************************头文件包含******************************************/
#include "bh1750.h"



/*************************************************************************************************
* 函数 ：   BH1750_WriteCommand() => 写入命令
*************************************************************************************************/
u8 BH1750_WriteCommand(u8 cmd)
{
	u8 error = 0;

	IIC_Start();
	IIC_Send_Byte(BH1750_ADD_W);
	error |= IIC_Wait_Ack();
	IIC_Send_Byte(cmd);
	error |= IIC_Wait_Ack();
	IIC_Stop();


	return error;
}

/*************************************************************************************************
* 函数 ：BH1750_ReadData() => 读取光线强度信息
*************************************************************************************************/
u8 BH1750_ReadData(float *rst,u8 mode)
{
	u8 error = 0;
	u8 rst_h = 0,rst_l = 0;
	u16 data = 0;

	error |= BH1750_ReadOriginalData(&rst_h,&rst_l);
	data = (rst_h << 8) | rst_l;

	switch(mode)
	{
		case BH1750_MODE_CH:
		case BH1750_MODE_OH:
			*rst = data / 1.2f;
			break;
		case BH1750_MODE_CL:
		case BH1750_MODE_OL:
			*rst = data / 1.2f;
			break;
		case BH1750_MODE_CH2:
		case BH1750_MODE_OH2:
			*rst = data / 2.4f;
			break;
		default:
			*rst = 0;
	}

	return error;
}

/*************************************************************************************************
* 函数 ：BH1750_ReadOriginalData(u8 *RST_H,u8 *RST_L,u8 mode) => 读取原始转换结果
* 参数 ：RST_H -> 原始结果高字节
*        RST_L -> 原始结果低字节
*************************************************************************************************/
u8 BH1750_ReadOriginalData(u8 *RST_H,u8 *RST_L)
{
  u8 error = 0;

  IIC_Start();
  IIC_Send_Byte(BH1750_ADD_R);
  error |= IIC_Wait_Ack();
  *RST_H = IIC_Read_Byte(1);			//先读取高字节
  *RST_L = IIC_Read_Byte(0);			//读取低字节
  IIC_Stop();

  return error;
}

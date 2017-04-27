/***********************************************************************************
  Filename:     bmp280.c

  Description:  大气压强传感器

***********************************************************************************/

/*********************************************头文件包含******************************************/
#include "bmp280.h"
#include "hal_mcu.h"





/*************************************************************************************************
* 函数 ：   BMP280_WriteRegister(u8 registerAddress,u8 data) => 写入寄存器
* 输入 ：   registerAddress - 寄存器地址
            data - 写入寄存器数据
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
* 函数 ：   BMP280_ReadRegister(u8 registerAdd,u8 *data) => 读取寄存器数据
* 输入 ：   registerAddress - 寄存器地址
            data - 读取寄存器数据
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
* 函数 ：   BMP280_ReadId() => 读取设备ID
* 输入 ：   *id - 读到的ID
*************************************************************************************************/
u8 BMP280_ReadId(u8 *id)
{
  u8 error = 0;
  
  error |= BMP280_ReadRegister(BMP_ID,id);
  
  return error;
}


/*************************************************************************************************
* 函数 ：   BMP280_Reset() => 软重启

*************************************************************************************************/
u8 BMP280_Reset()
{
  u8 error = 0;
  
  error |= BMP280_WriteRegister(BMP_RESET,0xB6);      //软重启写入0xB6，写入其他数字无效
  
  return error;
}

/*************************************************************************************************
* 函数 ：   BMP280_Init() => 初始化

*************************************************************************************************/
u8 BMP280_Init()
{
  u8 error = 0;
  u8 temp = 0;
  u16 t = 0;
  
  error |= BMP280_WriteRegister(BMP_CONFIG,0x00);
  error |= BMP280_WriteRegister(BMP_CTRL_MEAS,0x25);


  
  error |= BMP280_ReadRegister(BMP_DIG_T1+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T1,&temp);
  t |= temp;
  Bmp_Dig.t1 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_T2+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T2,&temp);
  t |= temp;
  Bmp_Dig.t2 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_T3+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T3,&temp);
  t |= temp;
  Bmp_Dig.t3 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P1+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P1,&temp);
  t |= temp;
  Bmp_Dig.p1 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P2+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P2,&temp);
  t |= temp;
  Bmp_Dig.p2 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P3+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P3,&temp);
  t |= temp;
  Bmp_Dig.p3 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P4+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P4,&temp);
  t |= temp;
  Bmp_Dig.p4 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P5+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P5,&temp);
  t |= temp;
  Bmp_Dig.p5 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P6+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P6,&temp);
  t |= temp;
  Bmp_Dig.p6 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P7+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P7,&temp);
  t |= temp;
  Bmp_Dig.p7 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P8+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P8,&temp);
  t |= temp;
  Bmp_Dig.p8 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P9+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P9,&temp);
  t |= temp;
  Bmp_Dig.p9 = t;
  
  return error;
}

/*************************************************************************************************
* 函数 ：   BMP280_TransStart() => 一次测量开始

*************************************************************************************************/
u8 BMP280_TransStart()
{
  u8 error = 0;
  
  error |= BMP280_WriteRegister(BMP_CTRL_MEAS,0x25);
  delay_ms(100);
  
  return error;
}


/*************************************************************************************************
* 函数 ：   BMP280_ReadResult() => 读取测量结果
* 参数 ：*data - 测量结果
*        *xdata - 分辨率
*        mode - 读取类型
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


/*************************************************************************************************
* 函数 ：   BMP280_ReadPressResult() => 读取压力真实结果，单位 1Pa，最高需要补1
* 说明 ：   读取数值为 0x8A7E(1000 1010 1110 0111)，实际大气压为0x18A7E(1 1000 1010 1110 0111)=101095Pa
*************************************************************************************************/
u8 BMP280_ReadPressResult(BMP280_U32_t *rst)
{
  BMP280_S32_t var1, var2, adc_T,adc_P,t_fine;
  BMP280_U32_t p;
  u16 data;
  u8 xdata;
  u8 error = 0;
  
  //读取温度信息，获取矫正参数
  error |= BMP280_ReadResult(&data,&xdata,BMP_TEMP); 
  adc_T = ((BMP280_S32_t)data << 4) | (xdata >> 4);
  var1 = ((((adc_T>>3) - ((BMP280_S32_t)Bmp_Dig.t1<<1))) * ((BMP280_S32_t)Bmp_Dig.t2)) >> 11;
  var2 = (((((adc_T>>4) - ((BMP280_S32_t)Bmp_Dig.t1)) * ((adc_T>>4) - ((BMP280_S32_t)Bmp_Dig.t1))) >> 12) * ((BMP280_S32_t)Bmp_Dig.t3)) >> 14;
  t_fine = var1 + var2;

  
  //读取压力信息
  error |= BMP280_ReadResult(&data,&xdata,BMP_PRESS);
  adc_P = ((BMP280_S32_t)data << 4) | (xdata >> 4);
  //矫正
  var1 = (((BMP280_S32_t)t_fine)>>1) - (BMP280_S32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BMP280_S32_t)Bmp_Dig.p6);
  var2 = var2 + ((var1*((BMP280_S32_t)Bmp_Dig.p5))<<1);
  var2 = (var2>>2)+(((BMP280_S32_t)Bmp_Dig.p4)<<16);
  var1 = (((Bmp_Dig.p3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BMP280_S32_t)Bmp_Dig.p2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((BMP280_S32_t)Bmp_Dig.p1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((BMP280_U32_t)(((BMP280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((BMP280_U32_t)var1);
  }
  else
  {
    p = (p / (BMP280_U32_t)var1) * 2;
  }
  var1 = (((BMP280_S32_t)Bmp_Dig.p9) * ((BMP280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((BMP280_S32_t)(p>>2)) * ((BMP280_S32_t)Bmp_Dig.p8))>>13;
  p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + Bmp_Dig.p7) >> 4));
  *rst = p;

  
  
  return error;
}


/*************************************************************************************************
* 函数 ：   BMP280_ReadTempResult() => 读取温度真实数据
* 参数 ：*data - 测量结果
*        *xdata - 分辨率
*        mode - 读取类型
*************************************************************************************************/
u8 BMP280_ReadTempResult(BMP280_S32_t *rst)
{
  BMP280_S32_t var1,T, var2, adc_T,t_fine;
  u16 data;
  u8 xdata;
  u8 error = 0;
  
  //读取温度信息，获取矫正参数
  error |= BMP280_ReadResult(&data,&xdata,BMP_TEMP); 
  adc_T = ((BMP280_S32_t)data << 4) | (xdata >> 4);
  var1 = ((((adc_T>>3) - ((BMP280_S32_t)Bmp_Dig.t1<<1))) * ((BMP280_S32_t)Bmp_Dig.t2)) >> 11;
  var2 = (((((adc_T>>4) - ((BMP280_S32_t)Bmp_Dig.t1)) * ((adc_T>>4) - ((BMP280_S32_t)Bmp_Dig.t1))) >> 12) * ((BMP280_S32_t)Bmp_Dig.t3)) >> 14;
  t_fine = var1 + var2;
  
  T = (t_fine * 5 + 128) >> 8;
  *rst = T;

  return error;
  
}


/*************************************************************************************************
* 函数 ：   BMP280_Update() => 更新修正量
*************************************************************************************************/
u8 BMP280_Update(void)
{
  u8 error = 0;
  u8 temp = 0;
  u16 t = 0;
  
  error |= BMP280_ReadRegister(BMP_DIG_T1+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T1,&temp);
  t |= temp;
  Bmp_Dig.t1 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_T2+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T2,&temp);
  t |= temp;
  Bmp_Dig.t2 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_T3+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_T3,&temp);
  t |= temp;
  Bmp_Dig.t3 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P1+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P1,&temp);
  t |= temp;
  Bmp_Dig.p1 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P2+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P2,&temp);
  t |= temp;
  Bmp_Dig.p2 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P3+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P3,&temp);
  t |= temp;
  Bmp_Dig.p3 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P4+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P4,&temp);
  t |= temp;
  Bmp_Dig.p4 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P5+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P5,&temp);
  t |= temp;
  Bmp_Dig.p5 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P6+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P6,&temp);
  t |= temp;
  Bmp_Dig.p6 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P7+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P7,&temp);
  t |= temp;
  Bmp_Dig.p7 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P8+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P8,&temp);
  t |= temp;
  Bmp_Dig.p8 = t;
  
  temp = 0;
  t = 0;
  error |= BMP280_ReadRegister(BMP_DIG_P9+1,&temp);
  t = temp << 8;
  error |= BMP280_ReadRegister(BMP_DIG_P9,&temp);
  t |= temp;
  Bmp_Dig.p9 = t;
  
  return error;
}
/*******************************************************************************
**
** Copyright (C) 2017 Zepto
** Contact: http://www.sunhx.cn
** E-mail: admin@sunhx.cn
**
*******************************************************************************/

#Wireless Sensor Networks
#农业温室无线传感节点

以 TI CC2530 为控制节点，通过 IIC 与光照强度传感器（OPT3001）、光线强度传感器（BH1750）、温湿度传感器（SHT20）和二氧化碳传感器（S100）通信进行数据读取，并通过 ZigBee 协议发送至上位机。


#Software
  IAR Embedded Workbench for 8051 Version 8.10.1

#Hardware
  TI CC2530
  TI OPT3001
  ROHM BHT1750FVI
  Sensirion SHT20
  ELT S100

#Document Tree
CC2530_V2.0
├──ide                      IAR工程文件
└──source                   程序文件
    ├──apps                 主程序
    |   ├──CC2530.c         main函数文件
    |   ├───delay.h         延时函数头文件，定义部分类型缩写
    |   ├───delay.c         延时函数代码
    |   ├───iic.h           IIC函数头文件，定义IIC_SCL/IIC_SDA
    |   ├───iic.c           IIC函数代码
    |   ├───opt3001.h       OPT3001头文件，定义各功能寄存器地址
    |   └───opt3001.c       OPT3001代码
    └──components           底层库函数

/***************************************************************************************
* 需要注意的参数值
* 全局变量：
*     pointType - 供电5V/3V
*     appMode - 工作角色 收/发
***************************************************************************************/



/*********************************************头文件包含******************************************/
#include <hal_led.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include "c51_gpio.h"
#include "c51_rtc.h"
#include "iic.h"
#include "opt3001.h"
#include "sht2x.h"
#include "bh1750.h"
#include "sht1x.h"
#include "max44009.h"
#include "bmp280.h"
#include "uart.h"

/************************************************常量定义****************************************/
#define TX                  1       // 发送模式
#define RX                  0       // 接收模式

#define VCC3V               0       //3v供电
#define VCC5V               1       //5v供电

#define TX_ADDR             0x2500  // 发送地址
#define RX_ADDR             0xBEEF  // 接收地址

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  步进为5MHZ
#define PAN_ID              0x2007  // PAN ID


#define SEND_LENGTH         30      // 发送数据每包的长度
#define CO2_SEND_LENGTH     2
#define RF_PKT_MAX_SIZE     30      // 数据包最大值


/************************************************全局变量****************************************/
static basicRfCfg_t basicRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // 接收缓存
static uint8    pTxData[SEND_LENGTH] = { 0x00 };  		 	// 需要发送的数据
static uint16   SendCnt = 0;                            // 计数发送的数据包数
static uint16   RecvCnt = 0;                            // 计数接收的数据包数
static uchar    Uart1_temp = 0;                         // 串口接收数据

static uchar    TransRst[SEND_LENGTH];                  // 需要发送的数据

uint32 counter = 0;         //定时器中断计数
uchar  timer_flag = 0;      //时间标志
uint32 TIMER_COUNTER = (uint32)1000 * 240;         //定时器定时时间，单位ms
u8 receive_trans_flag = 0;      //收到就转发
u8 CO2RecvFlag = 0;             //二氧化碳串口接收标志，0：正在接收；1：收到0x0D；2：收到0x0A，准备发送
u8 CO2RecvCount = 0;            //二氧化碳串口接收计数
u8 RF_TxCounter = 0;            //发送失败计数


uint8 pointType = VCC3V;
uint8 appMode = RX;


/************************************************函数声明****************************************/
static void Data_Init();                        //数据初始化
static u8 RF_SendPacket(void);                  // 发送函数
static void RF_RecvPacket(void);                  // 接收函数
static void RF_Initial(uint8 mode);               // RF初始化
static void RST_System(void);                     // 重启系统


/*************************************************************************************************
* 函数 ：   Data_Init() => 初始化数据
*************************************************************************************************/
static void Data_Init()
{
  u8 i;
  for(i = 0;i < 8;i++)
  {
    WireSensorData.AirPressure[i] = 0;
    WireSensorData.Illumination[i] = 0;
    WireSensorData.Temperature[i] = 0;
    WireSensorData.Humidity[i] = 0;
  }
  co2Concentration[0] = 0;
  co2Concentration[1] = 0;
}

/*************************************************************************************************
* 函数 ：   RF_Initial() => 初始化RF芯片
* 输入 ：   mode, =0,接收模式， else,发送模式
*************************************************************************************************/
static void RF_Initial(uint8 mode)
{
    // 设置地址
	if (RX == mode)     { basicRfConfig.myAddr = RX_ADDR; }
	else                { basicRfConfig.myAddr = TX_ADDR; }

    basicRfConfig.panId = PAN_ID;           // 设置节点PAN ID
    basicRfConfig.channel = RF_CHANNEL;     // 设置节点信道

    if (basicRfInit(&basicRfConfig) == FAILED)      { HAL_ASSERT(FALSE); }

//    halRfSetTxPower(1);                     // 设置输出功率为4dbm

    if (RX == mode)     { basicRfReceiveOn();  }
    else                { basicRfReceiveOff(); }
}

/*************************************************************************************************
* 函数 ：RST_System() => 重新启动系统，用于系统被唤醒后
*************************************************************************************************/
static void RST_System(void)
{
    halMcuInit();       // 重新初始化MCU
    halMcuWaitMs(5);    // 等待5ms，系统稳定
    RF_Initial(TX);     // 重新初始化RF
}

/*************************************************************************************************
* 函数 : BSP_RF_SendPacket() => 无线发送数据函数                            *
* 说明 ：Sendbuffer指向待发送的数据，length发送数据长度                     *
*************************************************************************************************/
static u8 RF_SendPacket(void)
{
    u8 states = 1;
    // 发送一包数据，并判断是否发送成功（收到应答）
    if(pointType == VCC3V)
    {
      if (!basicRfSendPacket(RX_ADDR, TransRst, SEND_LENGTH))
      {
          SendCnt++;
          halLedClear(1);         // LED闪烁，用于指示发送成功并且收到应答
          states = 0;
      }
      else
        states = 1;
    }
    else if(pointType == VCC5V)
    {
      if(!basicRfSendPacket(RX_ADDR, TransRst, CO2_SEND_LENGTH))
      {
        SendCnt++;
        halLedClear(1);
        states = 0;
      }
      else
        states = 1;
    }

    halMcuWaitMs(500);
    halLedSet(1);
    if(states)
    {
      if(RF_TxCounter++ > 14)
        states = 0;
    }
    if(states)
    {
      C51_RTC_EnterSleep(2);       // 系统进入模式PM2，低功耗，2s后被RTC唤醒
    }
    else
      C51_RTC_EnterSleep(240);
    RST_System();               // 重新初始化系统
    return states;
}

/*************************************************************************************************
* 函数 ：RF_RecvPacket() => 无线数据接收处理
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    u16 srcAddr = 0;

    if(!basicRfPacketIsReady())    // 检查模块是否已经可以接收下一个数据
      return;
    else
      basicRfPacketIsReadyIn(FALSE);

    // 把收到的数据复制到pRxData中
    if ((length=basicRfReceive(pRxData, RF_PKT_MAX_SIZE, NULL)) > 0)
    {
        // 判断接收数据是否正确
        if ((SEND_LENGTH==length))
        {
            RecvCnt++;
            srcAddr = basicRfGetTxSrcAddr();
            srcAddr = srcAddr & 0xff;
            srcAddr = srcAddr - 1;
            WireSensorData.error[srcAddr] = pRxData[0];
            WireSensorData.Illumination[srcAddr] = pRxData[1]<<8;
            WireSensorData.Illumination[srcAddr] |= pRxData[2];
            WireSensorData.Temperature[srcAddr] = pRxData[3]<<8;
            WireSensorData.Temperature[srcAddr] |= pRxData[4];
            WireSensorData.Humidity[srcAddr] = pRxData[5]<<8;
            WireSensorData.Humidity[srcAddr] |= pRxData[6];
            WireSensorData.AirPressure[srcAddr] = pRxData[7]<<8;
            WireSensorData.AirPressure[srcAddr] |= pRxData[8];
            //if(receive_trans_flag)            //调试使用
            //{
            //  Uart1_SendData_Test(srcAddr);
            //  Uart1_SendByte('\r');
            //  Uart1_SendByte('\n');
            //}
            // 闪烁LED，指示收到正确数据
            halLedClear(1);
            halMcuWaitMs(500);
            halLedSet(1);
        }
        else if(length == CO2_SEND_LENGTH)
        {
          srcAddr = basicRfGetTxSrcAddr();
          srcAddr = srcAddr & 0xff;
          srcAddr = srcAddr - 0x09;
          co2Concentration[srcAddr] = pRxData[0] << 8;
          co2Concentration[srcAddr] |= pRxData[1];


          // 闪烁LED，指示收到正确数据
          halLedClear(1);
          halMcuWaitMs(500);
          halLedSet(1);
        }
    }
}




/*************************************************************************************************
* 函数 ：Timer3_Init() => Time3初始化
* 说明 ：每次中断0.1ms
*************************************************************************************************/
void Timer3_Init(void)
{
  T3CC0 = 0xFA;     //计数125，1ms一次中断
  T3CCTL0 |= 0x04;
  T3CTL = 0xFE;     //32M时钟频率，128分频，中断使能，清除定时器，倒数模式
  T3IE = 1;
  EA = 1;
}

/*************************************************************************************************
* 函数 : main() => 主函数，程序入口
* 说明 ：发送方：每2s被RTC唤醒一次，醒来后发送一包数据，每包数据长度为5个字节，"123/r/n",蓝色LED
         闪烁表明发送成功，并且收到正确的应答数据。
         接收方: 如果接收到数据长度为5个字节，且前三个为“123”即数据接收正确，同时蓝色LED闪烁一次
         指示收到正确数据。
         注意：appMode = TX（发送程序）， =RX（接收程序）
*************************************************************************************************/
void main(void)
{
    u8 error = 0;
    /*
    //SHT2x
    u8 userRegister;  //用户寄存器
    nt16 sT;          //温度
    nt16 sRH;         //湿度
    */
    //SHT1x
    u8 stateRegister;  //用户寄存器
    u16 T,RH;         //温湿度
    //BMP280

    BMP280_U32_t rst;

    //co2
    u8 s100Rec[10];
    u8 uart1RxTemp = 0;
    u16 co2PPM = 0;



    halBoardInit();                                         // 初始外围设备

    IIC_Init();                                             // IIC初始化

    if(appMode == RX) Uart1_Init(UART_BAUD_115200,1);                    // Uart1初始化,115200
    else              Uart1_Init(UART_BAUD_38400,0);


    if (appMode == TX)              { C51_RTC_Initial(); }  // 初始化RTC，发送方需要睡眠
    else                            { Timer3_Init(); }

    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // 对硬件抽象层的rf进行初始化

    RF_Initial(appMode);                                    // 初始化RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO检测，判断IO是否有短接，断路

    //C51_GPIO_OffDetective();                                // 设置无关IO为输入，降低功耗


    if(appMode == TX && pointType == VCC3V)
    {
      halMcuWaitMs(3000);
      /*
      //OPT3001初始化
      error = 0;
      error = OPT3001_WriteRegister(OPT3001_REG_ADD_COF,0xCE10);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      */

      /*
      //SHT2x初始化
      error = 0;
      error |= SHT2x_SoftReset();
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      halMcuWaitMs(100);
      error = 0;
      error |= SHT2x_ReadUserRegister(&userRegister);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      error = 0;
      userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
      error |= SHT2x_WriteUserRegister(&userRegister);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      */

      //SHT1x初始化
      SHT1x_ReConnect();

      /*
      //BH1750初始化
      error = 0;
      error |= BH1750_WriteCommand(BH1750_MODE_CH);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      */

      //BMP280初始化
      BMP280_Init();
    }


    while (1)
    {
      if(appMode == RX)
      {
        RF_RecvPacket();        //RF接收
        //if(timer_flag && (!receive_trans_flag))     //调试使用
        if(timer_flag)
        {
          Uart1_SendData();
          Data_Init();
          timer_flag = 0;
        }
      }
      else if(appMode == TX)
      {
        RF_TxCounter = 0;
        if(pointType == VCC3V)
        {
          error = 0;

          /*
          //OPT3001
          error = 0;
          error |= OPT3001_ReadOriginalData(TransRst+1,TransRst+2);   //光照强度数据
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);
          */

          /*
          //bh1750
          //error = 0;
          error |= BH1750_ReadOriginalData(TransRst+1,TransRst+2);    //光照强度数据
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);
          */

          //MAX44009
          //error = 0;
          error |= MAX_ReadLightData(TransRst+1,TransRst+2);     //光照强度数据
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);


          /*
          //SHT2X
          //error = 0;
          //error |= SHT2x_SoftReset();
          //Uart1_SendByte(error);
          //error = 0;
          error |= SHT2x_ReadUserRegister(&userRegister);           //读取用户寄存器
          if( (userRegister & SHT2x_EOB_MASK) == SHT2x_EOB_ON )     //END OF BATTERY为1，电池电压过低
            error |= BATTERY_ALERT;
          Uart1_SendByte(error);
          //Uart1_SendByte(userRegister);
          //error = 0;
          //userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
          //error |= SHT2x_WriteUserRegister(&userRegister);
          //Uart1_SendByte(error);

          //error = 0;
          error |= SHT2x_MeasurePoll(TEMP,&sT);             //读取温度信息
          Uart1_SendByte(error);
          Uart1_SendByte(sT.s16.u8H);
          Uart1_SendByte(sT.s16.u8L);
          error = 0;
          error |= SHT2x_MeasurePoll(HUMIDITY,&sRH);        //读取湿度信息
          Uart1_SendByte(error);
          Uart1_SendByte(sRH.s16.u8H);
          Uart1_SendByte(sRH.s16.u8L);

          TransRst[3] = sT.s16.u8H;
          TransRst[4] = sT.s16.u8L;

          TransRst[5] = sRH.s16.u8H;
          TransRst[6] = sRH.s16.u8L;
          */


          //SHT1x
          //error = 0;
          error |= SHT1x_ReadTempResult(&T);           //读取温度信息
          TransRst[3] = (T >> 8) & 0xFF;
          TransRst[4] = T & 0xFF;
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[3]);
          //Uart1_SendByte(TransRst[4]);
          //Uart1_SendByte(checksum);

          //error = 0;
          error |= SHT1x_ReadRhResult(&RH);      //读取湿度信息
          TransRst[5] = (RH >> 8) & 0xFF;
          TransRst[6] = RH & 0xFF;
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[5]);
          //Uart1_SendByte(TransRst[6]);
          //Uart1_SendByte(checksum);

          //error = 0;
          error |= SHT1x_ReadStateRegister(&stateRegister);
          if(stateRegister & 0x40) error |= BATTERY_ALERT;
          //Uart1_SendByte(error);
          //Uart1_SendByte(stateRegister);



          //BMP280
          error |= BMP280_TransStart();
          error |= BMP280_ReadPressResult(&rst);
          TransRst[7] = (rst >> 8) & 0xFF;
          TransRst[8] = rst & 0xFF;
          //Uart1_SendByte(TransRst[7]);
          //Uart1_SendByte(TransRst[8]);


          TransRst[0] = error;

          while(RF_SendPacket());
          //RF_SendPacket();
        }
        else if(pointType == VCC5V)
        {
          if(URX1IF == 1)
          {
            URX1IF = 0;       //清中断标志
            uart1RxTemp = U1DBUF;

            if(uart1RxTemp == 0x0a)
            {
              if(CO2RecvFlag == 1)            //已经收到0x0d
              {
                if(CO2RecvCount == 10)        //已经收到10个有效数据
                {
                  co2PPM = 0;
                  if(s100Rec[0] >= '0' && s100Rec[0] <= '9')      //万位，应该没有
                    co2PPM += (s100Rec[0] - '0') * 10000;
                  if(s100Rec[1] >= '0' && s100Rec[1] <= '9')      //千位
                    co2PPM += (s100Rec[1] - '0') * 1000;
                  if(s100Rec[2] >= '0' && s100Rec[2] <= '9')      //百位
                    co2PPM += (s100Rec[2] - '0') * 100;
                  if(s100Rec[3] >= '0' && s100Rec[3] <= '9')      //十位
                    co2PPM += (s100Rec[3] - '0') * 10;
                  if(s100Rec[4] >= '0' && s100Rec[4] <= '9')      //个位
                    co2PPM += s100Rec[4] - '0';

                  TransRst[0] = co2PPM>>8;
                  TransRst[1] = co2PPM & 0xFF;

                  CO2RecvFlag = 2;      //准备发送
                }
                else
                {
                  CO2RecvCount = 0;
                  CO2RecvFlag = 0;
                }
              }
              else{
                CO2RecvFlag = 0;
                CO2RecvCount = 0;
              }
            }
            else if(uart1RxTemp == 0x0D)      //收到0x0d
            {
              if(CO2RecvFlag == 0)
              {
                CO2RecvFlag = 1;
                CO2RecvCount++;
              }
              else{
                CO2RecvFlag = 0;
                CO2RecvCount = 0;
              }
            }
            else      //收到数字或空格
            {
              if(CO2RecvCount < 9)
                s100Rec[CO2RecvCount++] = uart1RxTemp;
              else{
                CO2RecvCount = 0;
                CO2RecvFlag = 0;
              }
            }
          }

          if(CO2RecvFlag == 2){       //准备发送
            while(RF_SendPacket());
            CO2RecvFlag = 0;
            CO2RecvCount = 0;
          }
        }
      }

    }
}


/****************************************************************
* 函数 : UART0_ISR() => Uart1接收中断
* 说明 ：
****************************************************************/
#pragma vector = URX1_VECTOR
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//清中断标志
	Uart1_temp = U1DBUF;
        if(Uart1_temp & 0x80)
        {
          if(Uart1_temp & 0x01)
            receive_trans_flag = 1;
          else if(!(Uart1_temp & 0x01))
            receive_trans_flag = 0;
        }
        else
        {
          //TIMER_COUNTER = (uint32)10000 * (Uart1_temp & 0x7F);
          counter = 0;
        }
 }



#pragma vector = T3_VECTOR
__interrupt void T3_ISR(void)
{

  IRCON = 0x00;
  if(counter++ >= TIMER_COUNTER)
  {
    counter = 0;
    timer_flag = 1;
  }
}

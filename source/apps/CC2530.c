/***************************************************************************************
* 采集卡：
*       编译前定义 GATEWAY
*       默认上传时间 1min
*
* 传感器：
*       编译前定义 SENSOR
*       电压3v定义 VCC_3V     电压5V定义 VCC_5V
*       默认上传时间 2s
***************************************************************************************/



/*********************************************头文件包含******************************************/
#include <hal_led.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "awsn_rf.h"
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
//#define INCLUDE_PA 2592

#define TX                  1       // 发送模式
#define RX                  0       // 接收模式

#define VCC3V               0       //3v供电
#define VCC5V               1       //5v供电

#define TX_ADDR             0x2509  // 发送地址
#define RX_ADDR             0xBEEF  // 接收地址

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  步进为5MHZ
#define PAN_ID              0x2007  // PAN ID


#define SEND_LENGTH         12      // 发送数据每包的长度
#define CO2_SEND_LENGTH     3
#define RF_PKT_MAX_SIZE     30      // 数据包最大值

#define RSSI_H              -65     //RSSI上限
#define RSSI_L              -90      //RSSI下限


/************************************************全局变量****************************************/
static awsnRfCfg_t awsnRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // 接收缓存
static uint8    pTxData[SEND_LENGTH] = {0};  		// 需要发送的数据
static uint16   SendCnt = 0;                            // 计数发送的数据包数
static uint16   RecvCnt = 0;                            // 计数接收的数据包数
static uchar    Uart1_temp = 0;                         // 串口接收数据


uint32 counter = 0;         //定时器中断计数
uchar  timer_flag = 0;      //时间标志
uint32 TIMER_COUNTER = (uint32)1000 * 240;         //定时器定时时间，单位ms
u8 receive_trans_flag = 0;      //收到就转发
u8 CO2RecvFlag = 0;             //二氧化碳串口接收标志，0：正在接收；1：收到0x0D；2：收到0x0A，准备发送
u8 CO2RecvCount = 0;            //二氧化碳串口接收计数
u8 RF_TxCounter = 0;            //发送失败计数
u8 UartRecvData[4] = {0};       //串口接收数组，采集卡用
u8 UartRecvCounter = 0;         //串口接收数组计数，采集卡用

#ifdef VCC_5V
uint8 pointType = VCC5V;
#endif //VCC_5V
#ifdef VCC_3V
uint8 pointType = VCC3V;
#endif //VCC_3V
#ifdef GATEWAY
uint8 appMode = RX;
#endif //GATEWAY
#ifdef SENSOR
uint8 appMode = TX;
#endif //SENSOR


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
    if (RX == mode)     { awsnRfConfig.myAddr = RX_ADDR; }
    else                { awsnRfConfig.myAddr = TX_ADDR; }

    awsnRfConfig.panId = PAN_ID;           // 设置节点PAN ID
    awsnRfConfig.channel = RF_CHANNEL;     // 设置节点信道
    awsnRfConfig.txPower = 1;
#ifdef SENSOR
    awsnRfConfig.cycleTime = 0;
#endif
#ifdef GATEWAY
    awsnRfConfig.cycleTime = 6;
    TIMER_COUNTER = (uint32)10000 * awsnRfConfig.cycleTime;
#endif

    if (awsnRfInit(&awsnRfConfig) == FAILED)      { HAL_ASSERT(FALSE); }

    if(RX == mode)    { halRfSetTxPower(4); }                     // 设置输出功率为4dbm
    else              { halRfSetTxPower(1); }

    if (RX == mode)     { awsnRfReceiveOn();  }
    else                { awsnRfReceiveOff(); }
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
    if (!awsnRfSensorSendPacket(RX_ADDR, pTxData, SEND_LENGTH))
    {
        SendCnt++;
        halLedClear(1);         // LED闪烁，用于指示发送成功并且收到应答
        halMcuWaitMs(500);
        halLedSet(1);
        states = 0;
    }
    else
      states = 1;
  }
  else if(pointType == VCC5V)
  {
    if(!awsnRfSensorSendPacket(RX_ADDR, pTxData, CO2_SEND_LENGTH))
    {
      SendCnt++;
      halLedClear(1);
      halMcuWaitMs(500);
      halLedSet(1);
      states = 0;
    }
    else
      states = 1;
  }
  
  
  if(states)
  {
    if(RF_TxCounter++ == 3)
    {                                   //发送失败3次后加大发射功率
      if(awsnRfConfig.txPower != 2)
      {
        awsnRfConfig.txPower++;
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }
    else if(RF_TxCounter > 14)          //重复发送14次 30s 后
    {
      states = 0;
      awsnRfConfig.cycleTime = 30;      //可能采集卡未开，每 5min 尝试连接一次
    }
  }
  else{
    //成功收到应答信号后，提取信号质量
    if(awsnRfGetTxRssi() > RSSI_H)      //如果信号质量大于阈值
    {
      if(awsnRfConfig.txPower != 0)
      {
        awsnRfConfig.txPower--;       //降低发射功率
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }
    else if(awsnRfGetTxRssi() < RSSI_L)   //如果信号质量小于阈值
    {
      if(awsnRfConfig.txPower != 2)
      {
        awsnRfConfig.txPower++;         //增加发射功率
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }

    //修改采样周期
    awsnRfConfig.cycleTime = awsnRfGetTxLqi();
  }

  if(states)
  {
    C51_RTC_EnterSleep(2);       // 系统进入模式PM2，低功耗，2s后被RTC唤醒
  }
  else
    awsnRfSleep(awsnRfConfig.cycleTime);    //系统进入模式PM2，低功耗，睡眠时间由网关下发

  return states;
}

/*************************************************************************************************
* 函数 ：RF_RecvPacket() => 无线数据接收处理
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    u16 srcAddr = 0;
    int8 rssi = 0;

    if(!awsnRfPacketIsReady())    // 检查模块是否已经可以接收下一个数据
      return;

    
    length=awsnRfReceive(pRxData, RF_PKT_MAX_SIZE, &rssi);
    // 把收到的数据复制到pRxData中
    if (length > 0)
    {
        // 判断接收数据是否正确
        if ((SEND_LENGTH==length))
        {
            RecvCnt++;
            srcAddr = awsnRfGetTxSrcAddr();
            srcAddr = srcAddr & 0xff;
            srcAddr = srcAddr - 1;
            WireSensorData.error[srcAddr] = pRxData[0];
            WireSensorData.Illumination[srcAddr] = pRxData[1]<<8;
            WireSensorData.Illumination[srcAddr] |= pRxData[2];
            WireSensorData.Temperature[srcAddr] = pRxData[4]<<8;
            WireSensorData.Temperature[srcAddr] |= pRxData[5];
            WireSensorData.Humidity[srcAddr] = pRxData[7]<<8;
            WireSensorData.Humidity[srcAddr] |= pRxData[8];
            WireSensorData.AirPressure[srcAddr] = pRxData[10]<<8;
            WireSensorData.AirPressure[srcAddr] |= pRxData[11];
            halLedClear(1);
            halMcuWaitMs(500);
            halLedSet(1);
        }
        else if(length == CO2_SEND_LENGTH)
        {
          srcAddr = awsnRfGetTxSrcAddr();
          srcAddr = srcAddr & 0xff;
          srcAddr = srcAddr - 0x09;
          co2Concentration[srcAddr] = pRxData[1] << 8;
          co2Concentration[srcAddr] |= pRxData[2];


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

*************************************************************************************************/
void main(void)
{
    u8 error = 0;

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


    //外设初始化
    if(appMode == TX && pointType == VCC3V)
    {
      halMcuWaitMs(3000);

      //SHT1x初始化
      SHT1x_ReConnect();

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

          //MAX44009
          //error = 0;
          error |= MAX_ReadLightData(pTxData+1,pTxData+2);     //光照强度数据
          pTxData[0] = 0x10 + error;
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);

          //SHT1x
          error = 0;
          error |= SHT1x_ReadTempResult(&T);           //读取温度信息
          pTxData[4] = (T >> 8) & 0xFF;
          pTxData[5] = T & 0xFF;
          pTxData[3] = 0x20 + error;
          //Uart1_SendByte(error);
          //Uart1_SendByte(pTxData[3]);
          //Uart1_SendByte(pTxData[4]);
          //Uart1_SendByte(checksum);

          error = 0;
          error |= SHT1x_ReadRhResult(&RH);      //读取湿度信息
          pTxData[7] = (RH >> 8) & 0xFF;
          pTxData[8] = RH & 0xFF;
          //Uart1_SendByte(error);
          //Uart1_SendByte(pTxData[5]);
          //Uart1_SendByte(pTxData[6]);
          //Uart1_SendByte(checksum);

          //error = 0;
          error |= SHT1x_ReadStateRegister(&stateRegister);
          if(stateRegister & 0x40) error |= BATTERY_ALERT;
          pTxData[6] = 0x30 + error;
          //Uart1_SendByte(error);
          //Uart1_SendByte(stateRegister);

          //BMP280
          error = 0;
          error |= BMP280_TransStart();
          error |= BMP280_ReadPressResult(&rst);
          pTxData[9] = 0x40 + error;
          pTxData[10] = (rst >> 8) & 0xFF;
          pTxData[11] = rst & 0xFF;
          //Uart1_SendByte(TransRst[7]);
          //Uart1_SendByte(TransRst[8]);


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

                  pTxData[0] = 0;
                  pTxData[1] = co2PPM>>8;
                  pTxData[2] = co2PPM & 0xFF;

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


/*
void main(void)
{

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

    while(1)
    {
      if(appMode == TX)
      {
        RF_SendPacket();
      }
      else
      {
        RF_RecvPacket();
      }
    }

}
*/

/****************************************************************
* 函数 : UART0_ISR() => Uart1接收中断
* 说明 ：
****************************************************************/
#pragma vector = URX1_VECTOR
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//清中断标志
	Uart1_temp = U1DBUF;
        if(Uart1_temp == 0xA5)                  //起始位
        {
          UartRecvCounter = 0;
        }
        UartRecvData[UartRecvCounter++] = Uart1_temp;
        
        if(UartRecvCounter == 4)
        {
          if(!Uart_CheckSum(UartRecvData))
          {
            if(UartRecvData[1] == 0x01)
            {
              awsnRfConfig.cycleTime =  UartRecvData[2];
              TIMER_COUNTER = (uint32)10000 * awsnRfConfig.cycleTime;
            }
            else if(UartRecvData[1] == 0x00)
              RST_System();
          }
          
          UartRecvCounter = 0;
        }
 }


/****************************************************************
* 函数 : T3_ISR() => Timer3 中断
* 说明 ：仅采集卡使用，决定周期
****************************************************************/
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

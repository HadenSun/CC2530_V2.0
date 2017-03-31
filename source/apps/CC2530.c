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

/************************************************常量定义****************************************/
#define TX                  1       // 发送模式
#define RX                  0       // 接收模式

#define TX_ADDR             0x2520  // 发送地址                 
#define RX_ADDR             0xBEEF  // 接收地址

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  步进为5MHZ
#define PAN_ID              0x2007  // PAN ID

#define SEND_LENGTH         5       // 发送数据每包的长度
#define RF_PKT_MAX_SIZE     30      // 数据包最大值

/************************************************全局变量****************************************/
static basicRfCfg_t basicRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // 接收缓存
static uint8    pTxData[SEND_LENGTH] = { "123\r\n" };   // 需要发送的数据
static uint16   SendCnt = 0;                            // 计数发送的数据包数
static uint16   RecvCnt = 0;                            // 计数接收的数据包数

/************************************************函数声明****************************************/
static void RF_SendPacket(void);                  // 发送函数
static void RF_RecvPacket(void);                  // 接收函数
static void RF_Initial(uint8 mode);               // RF初始化
static void RST_System(void);                     // 重启系统
static void Uart1_Init(void);                     // Uart1初始化
static void Uart1_SendString(char *Date,int len); // Uart1发送字符串
static void Uart1_SendByte(uchar n);              // Uart1发送字节

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
    basicRfConfig.ackRequest = TRUE;        // 应答请求  
    
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
static void RF_SendPacket(void)
{ 
    // 发送一包数据，并判断是否发送成功（收到应答）
    if (!basicRfSendPacket(RX_ADDR, pTxData, SEND_LENGTH));
    {
        SendCnt++;
        halLedClear(1);         // LED闪烁，用于指示发送成功并且收到应答
    }
    halMcuWaitMs(500);
    halLedSet(1);
    C51_RTC_EnterSleep();       // 系统进入模式PM2，低功耗，2s后被RTC唤醒
    RST_System();               // 重新初始化系统  
}

/*************************************************************************************************
* 函数 ：RF_RecvPacket() => 无线数据接收处理
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    
    while (!basicRfPacketIsReady());    // 检查模块是否已经可以接收下一个数据

    // 把收到的数据复制到pRxData中
    if ((length=basicRfReceive(pRxData, RF_PKT_MAX_SIZE, NULL)) > 0) 
    {
        // 判断接收数据是否正确
        if ((SEND_LENGTH==length) && (pRxData[0]=='1')\
        && (pRxData[1]=='2') && (pRxData[2]=='3'))
        {
            RecvCnt++;
            
            // 闪烁LED，指示收到正确数据
            halLedClear(1);
            halMcuWaitMs(500);
            halLedSet(1);
        }  
    }
}


/*************************************************************************************************
* 函数 ：Uart1_Init() => Uart1初始化
* 说明 ：使用Uart1默认引脚，P0.4-TX，P0.5-RX
*************************************************************************************************/
void Uart1_Init(void)
{

    PERCFG = 0x00;                //UART1使用默认引脚 P0.4 P0.5
    P0SEL = 0x30;                 //P0.4 P0.5复用
    P2DIR |= 0x40;                //UART1优先

    U1CSR |= 0x80;                //选择UART模式     
    U1GCR |= 11;
    U1BAUD |= 216;                //波特率115200
    UTX1IF = 0;                   //清除中断标志
}

/*************************************************************************************************
* 函数 ：Uart1_SendString(char *Date,int len) => UART1发送字符串
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
* 函数 ：Uart1_SendByte(uchar n) => UART1发送字节
*************************************************************************************************/
void Uart1_SendByte(uchar n)
{
  U1DBUF = n;
  while(!UTX1IF);
  UTX1IF = 0;
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
    float rst;
    float hum = 0;
    float tem = 0;
    u8 error = 0;
    u8 userRegister;
    nt16 sT;
    nt16 sRH;
    uint8 appMode = TX;
    
    halBoardInit();                                         // 初始外围设备
    
    IIC_Init();                                             // IIC初始化
    
    Uart1_Init();                                            // Uart1初始化

    if (appMode == TX)              { C51_RTC_Initial(); }  // 初始化RTC
    
    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // 对硬件抽象层的rf进行初始化
       
    RF_Initial(appMode);                                    // 初始化RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO检测，判断IO是否有短接，断路
    
    //C51_GPIO_OffDetective();                                // 设置无关IO为输入，降低功耗
    
    
    error = 0;
    error = OPT3001_WriteRegister(OPT3001_REG_ADD_COF,0xCE10);
    Uart1_SendString("error:",6);
    Uart1_SendByte('0'+error);
    Uart1_SendByte('\r');
    Uart1_SendByte('\n');
    
    error = 0;
    error |= BH1750_WriteCommand(BH1750_MODE_CH);
    Uart1_SendString("error:",6);
    Uart1_SendByte('0'+error);
    Uart1_SendByte('\r');
    Uart1_SendByte('\n');
    while (1)
    {
        //if      (appMode == RX)     { RF_RecvPacket(); }    // 接收模块
        //else if (appMode == TX)     { RF_SendPacket(); }    // 发送模块
        // Role is undefined. This code should not be reached
        //HAL_ASSERT(FALSE);   
        
        rst = 0;
        tem = 0;
        hum = 0;
        
        //OPT3001
        error = 0;
	error |= OPT3001_ReadResult(&rst);
        Uart1_SendString("RST:",4);
        Uart1_SendByte((int)(rst/100)+'0');
        Uart1_SendByte((int)(rst/10)%10+'0');
        Uart1_SendByte((int)rst%10+'0');
        Uart1_SendByte('\r');
        Uart1_SendByte('\n');
        
        
      
        //SHT2X
        error = 0;
        error |= SHT2x_SoftReset();
        error |= SHT2x_ReadUserRegister(&userRegister);
        userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
        error |= SHT2x_WriteUserRegister(&userRegister);
        
        error |= SHT2x_MeasurePoll(TEMP,&sT);
        error |= SHT2x_MeasurePoll(HUMIDITY,&sRH);
        
        tem = SHT2x_CalcTemperatureC(sT.ul16);
        hum = SHT2x_CalcRH(sRH.ul16);
        
        Uart1_SendString("T:",2);
        Uart1_SendByte((uchar)tem/100+'0');
        Uart1_SendByte((uchar)tem/10%10+'0');
        Uart1_SendByte((uchar)tem%10+'0');
        Uart1_SendByte('\r');
        Uart1_SendByte('\n');
        
        
        //bh1750
        rst = 0;
        error = 0;
	error |= BH1750_ReadData(&rst,BH1750_MODE_CH);
        Uart1_SendString("L:",2);
        Uart1_SendByte((int)(rst/100)+'0');
        Uart1_SendByte((int)(rst/10)%10+'0');
        Uart1_SendByte((int)rst%10+'0');
        Uart1_SendByte('\r');
        Uart1_SendByte('\n');
        Uart1_SendByte('\r');
        Uart1_SendByte('\n');
        
        
        LED_B = ~LED_B;
        halMcuWaitMs(3000);
    }
}


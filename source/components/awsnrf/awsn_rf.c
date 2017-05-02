/***********************************************************************************
  Filename:     awsn_rf.c

  Description:  AWSN(Agriculture Wireless Sensor Network) RF 库

  编译时定义 GATEWAY/SENSOR

***********************************************************************************/

/*********************************************头文件包含******************************************/
#include "hal_int.h"
#include "hal_mcu.h"      //使用 halMcuWaiUs()
#include "hal_rf.h"
#include "awsn_rf.h"
#include "util.h"
#include "string.h"
#include "c51_rtc.h"
#include <ioCC2530.h>


/************************************************常量定义****************************************/
//定义帧长度
#define PKT_LEN_ENTER   8   //入网帧长度，携带IEEE地址
#define PKT_LEN_SYN     1   //同步帧长度，暂定1字节
#define PKT_LEN_ACK     2   //确认帧长度，暂定2字节
#define PKT_LEN_ADD     4   //地址分配帧长度，4字节
#define PKT_LEN_DATA    3   //数据帧每个传感器数据长度

//MAC部分长度
#define MAC_LEN_HEAD        (2+1+2+2+2+1+1)   //MAC头长度
#define MAC_LEN_OVERHEAD    (MAC_LEN_HEAD+2)      //MAC头+尾长度

//FCF帧控制域
#define AWSN_RF_FCF_ACK     0x8861      //要求应答
#define AWSN_RF_FCF_NACK    0x8841      //不需应答

//CRC校验位
#define AWSN_RF_CRC_OK_BM    0x80

#define AWSN_RF_PLD_LEN_MASK  0x7F      //PHY层长度有效位
#define AWSN_RF_LEN_MAX_PAYLOAD   (127 - MAC_LEN_OVERHEAD)

//等待应答包的时间
//每字节经调制需32个信号，每个信号约0.5us
#define AWSN_RF_ACK_DURATION      (0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 1) + (2)))
#define AWSN_RF_SYMBOL_DURATION   (32 * 0.5)


/************************************************结构定义****************************************/
//接收结构
typedef struct {
  uint8 seqNumber;          //帧序号
  uint16 srcAddr;           //发送地址
  uint16 srcPanId;          //PAN ID
  uint8 txPower;            //发射功率 0:-3db 1:1db 2:4db
  uint8 cycleTime;          //发送周期，单位10s
  uint8 length;             //接收长度
  uint8* pPayload;          //负载指针
  int8 rssi;                //接收信号强度
  volatile uint8 isReady;   //是否完成有效接收
  uint8 status;             //状态
} awsnRfRxInfo_t;

//发送结构
typedef struct {
  uint8 txSeqNumber;            //发送帧序号
  volatile uint8 ackReceived;   //是否接收应答
  int8 rssi;                    //应答信号回馈的rssi
  uint8 lqi;                    //暂改为周期，单位10s
  uint32 frameCounter;          //帧计数
  uint8 receiveOn;              //接收应答信号标志
} awsnRfTxState_t;


//包头
typedef struct {
  uint8 packetLength;       //帧长度
  uint8 fcf0;               
  uint8 fcf1;
  uint8 seqNumber;          //帧序号
  uint16 panId;             //PanId
  uint16 destAddr;          //目的地址
  uint16 srcAddr;           //源地址
  uint8 txPower;            //发送功率
  uint8 cycleTime;          //发送周期，单位10s
}awsnRfPktHdr_t;            //包结构为 awsnRfPktHdr_t + payload + CRC


/************************************************全局变量****************************************/
static awsnRfRxInfo_t rxi = {0xFF};
static awsnRfTxState_t txState = {0xFF};

static awsnRfCfg_t* pConfig;
static uint8 txMpdu[127];
static uint8 rxMpdu[128];



/*************************************************************************************************
* 函数 ：  awsnRfBuildHeader
* 介绍 ：  构建MAC部分帧头
* 输入 ：  buffer - 帧头内容指针
          destAddr - 目的地址
          payloadLength - MAC部分负载长度
*************************************************************************************************/
static void awsnRfBuildHeader(uint8* buffer,uint16 destAddr, uint8 payloadLength)
{
  awsnRfPktHdr_t *pHdr;
  uint16 fcf;

  pHdr = (awsnRfPktHdr_t*)buffer;

  pHdr->packetLength = payloadLength + MAC_LEN_OVERHEAD;
  #ifdef GATEWAY
  //采集卡部分发送的数据不需要应答
  fcf = AWSN_RF_FCF_NACK;
  #else
  //传感器节点发送数据需要应答
  fcf = AWSN_RF_FCF_ACK;
  #endif
  pHdr->fcf0 = LO_UINT16(fcf);
  pHdr->fcf1 = HI_UINT16(fcf);
  pHdr->seqNumber = txState.txSeqNumber;
  pHdr->panId = pConfig->panId;
  pHdr->destAddr = destAddr;
  pHdr->srcAddr = pConfig->myAddr;
  pHdr->txPower = pConfig->txPower;
  pHdr->cycleTime = pConfig->cycleTime;

  UINT16_HTON(pHdr->panId);
  UINT16_HTON(pHdr->destAddr);
  UINT16_HTON(pHdr->srcAddr);

}


/*************************************************************************************************
* 函数 ：  awsnRfBuildMpdu
* 介绍 ：  构建PHY部分帧内容
* 输入 ：  destAddr - 目的地址
          pPayload - MAC部分负载内容指针
          payloadLength - MAC部分负载长度
* 返回值： PHY部分帧长度
*************************************************************************************************/
static uint8 awsnRfBuildMpdu(uint16 destAddr, uint8 *pPayload, uint8 payloadLength)
{
  uint8  n;

  awsnRfBuildHeader(txMpdu,destAddr,payloadLength);

  for(n = 0; n < payloadLength; n++)
  {
    txMpdu[MAC_LEN_HEAD+1+n] = pPayload[n];
  }

  return MAC_LEN_HEAD + payloadLength + 1;
}


/*************************************************************************************************
* 函数 ：  awsnRfRxInterrupt
* 介绍 ：  无线接收中断
*************************************************************************************************/
static void awsnRfRxInterrupt(void)
{
  awsnRfPktHdr_t *pHdr;
  uint8 *pStatusWord;

  pHdr = (awsnRfPktHdr_t*)rxMpdu;

  //清除中断标志并禁止新的RX接收中断
  halRfDisableRxInterrupt();

  //允许其他中断
  halIntOn();

  //读取接收长度
  halRfReadRxBuf(&pHdr->packetLength,1);          //读取RxBuf第一字节内容，MAC层总长度
  pHdr->packetLength &= AWSN_RF_PLD_LEN_MASK;     //忽略最高bit，长度有效值

  //读取所有数据
  halRfReadRxBuf(&rxMpdu[1],pHdr->packetLength);
  #ifdef SENSOR
  //传感器节点
  if(pHdr->packetLength == PKT_LEN_ACK + MAC_LEN_OVERHEAD) {
    //如果是应答帧
    rxi.pPayload = rxMpdu + MAC_LEN_HEAD + 1;
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ACK + 1;
    if((pStatusWord[1] & AWSN_RF_CRC_OK_BM)) {
      txState.rssi = rxi.pPayload[0];    //读取采集卡返回的RSSI
      txState.lqi = rxi.pPayload[1];   //读取采集卡返回的LQI
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ACK;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_ADD + MAC_LEN_OVERHEAD) {
    //地址分配帧
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ADD;
    if((*pStatusWord & AWSN_RF_CRC_OK_BM)) {
      pConfig->panId = rxMpdu[MAC_LEN_HEAD];
      pConfig->myAddr = rxMpdu[MAC_LEN_HEAD+1];
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ADD;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_SYN + MAC_LEN_OVERHEAD) {
    //同步帧
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_SYN;
    if(*pStatusWord & AWSN_RF_CRC_OK_BM)
    {
      pConfig->cycleTime = rxMpdu[MAC_LEN_HEAD];
      rxi.status = AWSN_SYN;
    }
  }
  #endif

  #ifdef GATEWAY
  //采集卡节点
  if(pHdr->packetLength == PKT_LEN_ENTER + MAC_LEN_OVERHEAD) {
    //入网帧
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ENTER;
    if(*pStatusWord & AWSN_RF_CRC_OK_BM) {
      rxi.status = AWSN_ENTER;
    }
  }
  else {
    //数据帧
    rxi.length = pHdr->packetLength - MAC_LEN_OVERHEAD;
    rxi.srcAddr = pHdr->srcAddr;
    rxi.pPayload = rxMpdu + MAC_LEN_HEAD + 1;
    pStatusWord = rxi.pPayload + rxi.length;
    rxi.rssi = pStatusWord[0];

    if((pStatusWord[1] & AWSN_RF_CRC_OK_BM)) {
      rxi.isReady = TRUE;
      rxi.status = AWSN_DATA;
      
      pStatusWord[1] = pConfig->cycleTime;
      awsnRfGatewaySendPacket(rxi.srcAddr,pStatusWord,2);
    }
    rxi.seqNumber = pHdr->seqNumber;
  }
  #endif

  //RX接收中断使能
  halIntOff();
  halRfEnableRxInterrupt();

}



/*************************************************************************************************
* 函数 ：  awsnRfInit
* 介绍 ：  RF初始化
* 参数 ：  pRfConfig - 配置结构指针
*************************************************************************************************/
uint8 awsnRfInit(awsnRfCfg_t* pRfConfig)
{
  if(halRfInit() == FAILED)
    return FAILED;

  halIntOff();

  pConfig = pRfConfig;
  rxi.pPayload = NULL;

  txState.frameCounter = 0;
  txState.receiveOn = TRUE;

  //设置频道
  halRfSetChannel(pConfig->channel);

  //写入短地址
  halRfSetShortAddr(pConfig->myAddr);
  halRfSetPanId(pConfig->panId);

  //读取MAC地址
  awsnRfGetMacAddr();

  //设置接收中断
  halRfRxInterruptConfig(awsnRfRxInterrupt);

  halIntOn();

  return SUCCESS;
}


/*************************************************************************************************
* 函数 ：  awsnRfSensorSendPacket
* 介绍 ：  RF 传感器模块发送数据
* 参数 ：  destAddr - 目的地址
*         pPayload - 负载内容指针
*         length - 负载长度
* 返回值：  SUCCESS or FAILED
*************************************************************************************************/
uint8 awsnRfSensorSendPacket(uint16 destAddr,uint8 *pPayload,uint8 length)
{
  uint8 mpduLength;
  uint8 status;

  //打开RF接收
  if(!txState.receiveOn) {
    halRfReceiveOn();
  }

  //检查包长度
  length = min(length,AWSN_RF_LEN_MAX_PAYLOAD);

  //等待发送成功
  halRfWaitTransceiverReady();

  //关闭RF接收中断
  halRfDisableRxInterrupt();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);

  //打开接收中断，等待应答
  halRfEnableRxInterrupt();
  
  //等待应答消息
  txState.ackReceived = FALSE;

  //发送数据
  if(halRfTransmit() != SUCCESS) {
    status = FAILED;
  }

  //超时设置，包含12个传输时间，应答包的传送时间，和一些额外时间
  //halMcuWaitUs((12 * AWSN_RF_SYMBOL_DURATION) + (AWSN_RF_ACK_DURATION) + (2 * AWSN_RF_SYMBOL_DURATION) + 100);
  halMcuWaitMs(2);

  status = txState.ackReceived? SUCCESS : FAILED;

  if(!txState.receiveOn) {
    halRfReceiveOff();
  }


  if(status == SUCCESS)
  {
    txState.txSeqNumber++;
  }

  return status;
}



/*************************************************************************************************
* 函数 ：  awsnRfGatewaySendPacket
* 介绍 ：  RF 网关模块发送数据
* 参数 ：  destAddr - 目的地址
*         pPayload - 负载内容指针
*         length - 负载长度
* 返回值：  SUCCESS or FAILED
*************************************************************************************************/
uint8 awsnRfGatewaySendPacket(uint16 destAddr,uint8 *pPayload,uint8 length)
{
  uint8 mpduLength;
  uint8 status;


  //检查包长度
  length = min(length,AWSN_RF_LEN_MAX_PAYLOAD);

  //等待发送成功
  halRfWaitTransceiverReady();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);


  //发送数据
  if(halRfTransmit() != SUCCESS) {
    status = FAILED;
  }
  else {
    status = SUCCESS;
  }


  if(status == SUCCESS)
  {
    txState.txSeqNumber++;
  }

  return status;
}


/*************************************************************************************************
* 函数 ：  awsnRfPacketIsReady
* 介绍 ：  数据接收是否完成
* 返回值：
*************************************************************************************************/
uint8 awsnRfPacketIsReady(void)
{
  return rxi.isReady;
}


/*************************************************************************************************
* 函数 ：  awsnRfRecive
* 介绍 ：  复制负载内容到buffer中
* 参数 ：  pRxData - buffer指针
*         len - 负载长度
*         pRssi - 接收信号强度
* 返回值：
*************************************************************************************************/
uint8 awsnRfReceive(uint8 *pRxData,uint8 len, int8* pRssi)
{
  halIntOff();
  memcpy(pRxData,rxi.pPayload,min(rxi.length,len));
  if(pRssi != NULL) {
    if(rxi.rssi < 128) {
      *pRssi = rxi.rssi - halRfGetRssiOffset();
    }
    else {
      *pRssi = (rxi.rssi - 256) - halRfGetRssiOffset();
    }
  }

  rxi.isReady = FALSE;
  halIntOn();

  return min(rxi.length,len);
}



/*************************************************************************************************
* 函数 ：  awsnRfRecieveOn
* 介绍 ：  接收打开
*************************************************************************************************/
void awsnRfReceiveOn(void)
{
  txState.receiveOn = TRUE;
  halRfReceiveOn();
}


/*************************************************************************************************
* 函数 ：  awsnRfRecieveOff
* 介绍 ：  接收关闭
*************************************************************************************************/
void awsnRfReceiveOff(void)
{
  txState.receiveOn = FALSE;
  halRfReceiveOff();
}


/*************************************************************************************************
* 函数 ：  awsnRfMacAddr
* 介绍 ：  获取Mac地址
*************************************************************************************************/
void awsnRfGetMacAddr(void)
{
  uint8 *macaddrptr = (uint8 *)(P_INFOPAGE+HAL_INFOP_IEEE_OSET);

  for(int i = 0; i < 8; i++)
  {
    pConfig->macAddr[i] = macaddrptr[i];
  }
}


/*************************************************************************************************
* 函数 ：  awsnRfGetTxRssi
* 介绍 ：  获取发射信号质量
* 返回值： 网关收到的传感器的RSSI
*************************************************************************************************/
int8 awsnRfGetTxRssi(void)
{
  int8 rssi;

  if(txState.rssi < 128) {
    rssi = txState.rssi - halRfGetRssiOffset();
  }
  else {
    rssi = (txState.rssi - 256) - halRfGetRssiOffset();
  }
  
  return rssi;
}


/*************************************************************************************************
* 函数 ：  awsnRfGetTxLqi
* 介绍 ：  获取发射信号质量
* 返回值： 网关收到的传感器的LQI
*************************************************************************************************/
uint8 awsnRfGetTxLqi(void)
{
  return txState.lqi;
}


/*************************************************************************************************
* 函数 ：  awsnRfSleep
* 介绍 ：  休眠
* 参数 ：  sleepTime - 睡眠时间，单位10s
*************************************************************************************************/
void awsnRfSleep(uint8 sleepTime)
{
  uint16 sleepTotalTime = sleepTime * 10 - 5;
  uint8 i;

  if(sleepTime > 25)
  {
    for((i = sleepTotalTime / 255); i > 0; i--)
    {
      C51_RTC_EnterSleep(255);
      halMcuInit();       // 重新初始化MCU
      halMcuWaitMs(5);    // 等待5ms，系统稳定
      halRfSetTxPower(pConfig->txPower);
      awsnRfReceiveOff();
    }
    C51_RTC_EnterSleep(sleepTotalTime % 255);
    halMcuInit();       // 重新初始化MCU
    halMcuWaitMs(5);    // 等待5ms，系统稳定
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
  else if(sleepTime != 0){
    C51_RTC_EnterSleep(sleepTime * 10);   //睡眠
    halMcuInit();       // 重新初始化MCU
    halMcuWaitMs(5);    // 等待5ms，系统稳定
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
  else if(sleepTime == 0)
  {
    C51_RTC_EnterSleep(2);   //睡眠
    halMcuInit();       // 重新初始化MCU
    halMcuWaitMs(5);    // 等待5ms，系统稳定
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
}


/*************************************************************************************************
* 函数 ：  awsnRfGetTxSrcAddr
* 介绍 ：  获取发送Addr
* 返回值： 发送方srcaddr
*************************************************************************************************/
uint8 awsnRfGetTxSrcAddr(void)
{
  return rxi.srcAddr;
}

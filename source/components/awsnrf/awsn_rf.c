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


/************************************************常量定义****************************************/
//定义帧长度
#define PKT_LEN_ENTER   8   //入网帧长度，携带IEEE地址
#define PKT_LEN_SYN     1   //同步帧长度，暂定1字节
#define PKT_LEN_ACK     2   //确认帧长度，暂定2字节
#define PKT_LEN_ADD     4   //地址分配帧长度，4字节
#define PKT_LEN_DATA    3   //数据帧每个传感器数据长度

//MAC部分长度
#define MAC_LEN_HEAD        (2+1+2+2+2+1+1)   //MAC头长度
#define MAC_LEN_OVERHEAD    MAC_LEN_HEAD+2      //MAC头+尾长度

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
  uint8 seqNumber;
  uint16 srcAddr;
  uint16 srcPanId;
  uint8 txPower;
  uint8 cycleTime;
  uint8 length;
  uint8* pPayload;
  int8 rssi;
  volatile uint8 isReady;
  uint8 status;
} awsnRfRxInfo_t;

//发送结构
typedef struct {
  uint8 txSeqNumber;
  volatile uint8 ackReceived;
  uint8 rssi;
  uint8 lqi;
  uint32 frameCounter;
  uint8 receiveOn;
} awsnRfTxState_t;


//包头
typedef struct {
  uint8 packetLength;
  uint8 fcf0;
  uint8 fcf1;
  uint8 seqNumber;
  uint16 panId;
  uint16 destAddr;
  uint16 srcAddr;
  uint8 txPower;
  uint8 cycleTime;
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
  #endif
  #ifdef SENSOR
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
  uint8 hdrLength, n;

  awsnRfBuildHeader(txMpdu,destAddr,payloadLength);

  for(n = 0; n < payloadLength; n++)
  {
    txMpdu[MAC_LEN_HEAD+n] = pPayload[n];
  }

  return MAC_LEN_HEAD + payloadLength;
}


/*************************************************************************************************
* 函数 ：  awsnRfRxInterrupt
* 介绍 ：  无线接收中断
*************************************************************************************************/
static void awsnRfRxInterrupt(void)
{
  awsnRfPktHdr_t *pHdr;
  uint8 *pStateWord;

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
    pStateWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ACK;
    if((*pStateWord & AWSN_RF_CRC_OK_BM)&&(pHdr->seqNumber == txState.txSeqNumber) {
      txState.rssi = rxMpdu[MAC_LEN_HEAD];    //读取采集卡返回的RSSI
      txState.lqi = rxMpdu[MAC_LEN_HEAD+1];   //读取采集卡返回的LQI
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ACK;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_ADD + MAC_LEN_OVERHEAD) {
    //地址分配帧
    pStateWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ADD;
    if((*pStateWord & AWSN_RF_CRC_OK_BM)&&(pHdr->seqNumber == txState.txSeqNumber)) {
      qConfig->panId = rxMpdu[MAC_LEN_HEAD]
      qConfig->myAddr = rxMpdu[MAC_LEN_HEAD+1];
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ADD;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_SYN + MAC_LEN_OVERHEAD) {
    //同步帧
    pStateWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_SYN;
    if(*pStateWord & AWSN_RF_CRC_OK_BM)
    {
      qConfig->cycleTime = rxMpdu[MAC_LEN_HEAD];
      rxi.status = AWSN_SYN;
    }
  }
  #endif

  #ifdef GATEWAY
  //采集卡节点
  if(pHdr->packetLength == PKT_LEN_ENTER + MAC_LEN_OVERHEAD) {
    //入网帧
    pStateWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ENTER;
    if(*pStateWord & AWSN_RF_CRC_OK_BM) {
      rxi.status = AWSN_ENTER;
    }
  }
  else {
    //数据帧
    rxi.length = pHdr->packetLength - MAC_LEN_OVERHEAD;
    rxi.srcAddr = pHdr->srcAddr;
    rxi.pPayload = rxMpdu + MAC_LEN_HEAD;
    pStateWord = rxi.pPayload + rxi.length;
    rxi.rssi = pStateWord[0];

    if((pStateWord[1] & AWSN_RF_CRC_OK_BM) && (rxi.seqNumber != pHdr->seqNumber)) {
      rxi.isReady = TRUE;
      rxi.status = AWSN_DATA;
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
  txi.pPayload = NULL;

  txState.frameCounter = 0;

  //设置频道
  halRfSetChannel(pConfig->channel);

  //写入短地址
  halRfSetShortAddr(pConfig->myAddr);
  halRfPanId(pConfig->panId);

  //读取MAC地址
  awsnRfGetMacAddr();

  //设置接收中断
  halRfInterrruptConfig(awsnRfRxInterrupt);

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
  halRfWaitTransceiveReady();

  //关闭RF接收中断
  halRfDisableRxInterrupt();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);

  //打开接收中断，等待应答
  halRfEnableRxInterrupt();

  //发送数据
  if(halRfTransmit() != SUCCESS) {
    status = FAILED;
  }

  //等待应答消息
  txState.ackReceived = FALSE;

  //超时设置，包含12个传输时间，应答包的传送时间，和一些额外时间
  halMcuWaiUs((12 * AWSN_RF_SYMBOL_DURATION) + (AWSN_RF_ACK_DURATION) + (2 * AWSN_RF_SYMBOL_DURATION) + 10);

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

  //打开RF接收
  if(!txState.receiveOn) {
    halRfReceiveOn();
  }

  //检查包长度
  length = min(length,AWSN_RF_LEN_MAX_PAYLOAD);

  //等待发送成功
  halRfWaitTransceiveReady();

  //关闭RF接收中断
  halRfDisableRxInterrupt();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);


  //发送数据
  if(halRfTransmit() != SUCCESS) {
    status = FAILED;
  }
  else {
    status = SUCCESS;
  }


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
* 函数 ：  awsnRfPacketIsReady
* 介绍 ：  数据接收是否完成
* 返回值：
*************************************************************************************************/
uint8 awsnRfPacketIsReady(void)
{
  return rxi.isReady;
}


/*************************************************************************************************
* 函数 ：  awsnRfRecieve
* 介绍 ：  复制负载内容到buffer中
* 参数 ：  pRxData - buffer指针
*         len - 负载长度
*         pRssi - 接收信号强度
* 返回值：
*************************************************************************************************/
uint8 awsnRfRecieve(uint8 *pRxData,uint8 len, int8* pRssi)
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
void awsnRfRecieveOn(void)
{
  txState.receiveOn = TRUE;
  halRfReceiveOn();
}


/*************************************************************************************************
* 函数 ：  awsnRfRecieveOff
* 介绍 ：  接收关闭
*************************************************************************************************/
void awsnRfRecieveOff(void)
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
    awsnRfCfg_t.macAddr[i] = macaddrptr[i];
  }
}

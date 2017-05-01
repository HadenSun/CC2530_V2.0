/***********************************************************************************
  Filename:     awsn_rf.c

  Description:  AWSN(Agriculture Wireless Sensor Network) RF ��

  ����ʱ���� GATEWAY/SENSOR

***********************************************************************************/

/*********************************************ͷ�ļ�����******************************************/
#include "hal_int.h"
#include "hal_mcu.h"      //ʹ�� halMcuWaiUs()
#include "hal_rf.h"
#include "awsn_rf.h"
#include "util.h"
#include "string.h"
#include "c51_rtc.h"
#include <ioCC2530.h>


/************************************************��������****************************************/
//����֡����
#define PKT_LEN_ENTER   8   //����֡���ȣ�Я��IEEE��ַ
#define PKT_LEN_SYN     1   //ͬ��֡���ȣ��ݶ�1�ֽ�
#define PKT_LEN_ACK     2   //ȷ��֡���ȣ��ݶ�2�ֽ�
#define PKT_LEN_ADD     4   //��ַ����֡���ȣ�4�ֽ�
#define PKT_LEN_DATA    3   //����֡ÿ�����������ݳ���

//MAC���ֳ���
#define MAC_LEN_HEAD        (2+1+2+2+2+1+1)   //MACͷ����
#define MAC_LEN_OVERHEAD    (MAC_LEN_HEAD+2)      //MACͷ+β����

//FCF֡������
#define AWSN_RF_FCF_ACK     0x8861      //Ҫ��Ӧ��
#define AWSN_RF_FCF_NACK    0x8841      //����Ӧ��

//CRCУ��λ
#define AWSN_RF_CRC_OK_BM    0x80

#define AWSN_RF_PLD_LEN_MASK  0x7F      //PHY�㳤����Чλ
#define AWSN_RF_LEN_MAX_PAYLOAD   (127 - MAC_LEN_OVERHEAD)

//�ȴ�Ӧ�����ʱ��
//ÿ�ֽھ�������32���źţ�ÿ���ź�Լ0.5us
#define AWSN_RF_ACK_DURATION      (0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 1) + (2)))
#define AWSN_RF_SYMBOL_DURATION   (32 * 0.5)


/************************************************�ṹ����****************************************/
//���սṹ
typedef struct {
  uint8 seqNumber;          //֡���
  uint16 srcAddr;           //���͵�ַ
  uint16 srcPanId;          //PAN ID
  uint8 txPower;            //���书�� 0:-3db 1:1db 2:4db
  uint8 cycleTime;          //�������ڣ���λ10s
  uint8 length;             //���ճ���
  uint8* pPayload;          //����ָ��
  int8 rssi;                //�����ź�ǿ��
  volatile uint8 isReady;   //�Ƿ������Ч����
  uint8 status;             //״̬
} awsnRfRxInfo_t;

//���ͽṹ
typedef struct {
  uint8 txSeqNumber;            //����֡���
  volatile uint8 ackReceived;   //�Ƿ����Ӧ��
  int8 rssi;                    //Ӧ���źŻ�����rssi
  uint8 lqi;                    //�ݸ�Ϊ���ڣ���λ10s
  uint32 frameCounter;          //֡����
  uint8 receiveOn;              //����Ӧ���źű�־
} awsnRfTxState_t;


//��ͷ
typedef struct {
  uint8 packetLength;       //֡����
  uint8 fcf0;               
  uint8 fcf1;
  uint8 seqNumber;          //֡���
  uint16 panId;             //PanId
  uint16 destAddr;          //Ŀ�ĵ�ַ
  uint16 srcAddr;           //Դ��ַ
  uint8 txPower;            //���͹���
  uint8 cycleTime;          //�������ڣ���λ10s
}awsnRfPktHdr_t;            //���ṹΪ awsnRfPktHdr_t + payload + CRC


/************************************************ȫ�ֱ���****************************************/
static awsnRfRxInfo_t rxi = {0xFF};
static awsnRfTxState_t txState = {0xFF};

static awsnRfCfg_t* pConfig;
static uint8 txMpdu[127];
static uint8 rxMpdu[128];



/*************************************************************************************************
* ���� ��  awsnRfBuildHeader
* ���� ��  ����MAC����֡ͷ
* ���� ��  buffer - ֡ͷ����ָ��
          destAddr - Ŀ�ĵ�ַ
          payloadLength - MAC���ָ��س���
*************************************************************************************************/
static void awsnRfBuildHeader(uint8* buffer,uint16 destAddr, uint8 payloadLength)
{
  awsnRfPktHdr_t *pHdr;
  uint16 fcf;

  pHdr = (awsnRfPktHdr_t*)buffer;

  pHdr->packetLength = payloadLength + MAC_LEN_OVERHEAD;
  #ifdef GATEWAY
  //�ɼ������ַ��͵����ݲ���ҪӦ��
  fcf = AWSN_RF_FCF_NACK;
  #else
  //�������ڵ㷢��������ҪӦ��
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
* ���� ��  awsnRfBuildMpdu
* ���� ��  ����PHY����֡����
* ���� ��  destAddr - Ŀ�ĵ�ַ
          pPayload - MAC���ָ�������ָ��
          payloadLength - MAC���ָ��س���
* ����ֵ�� PHY����֡����
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
* ���� ��  awsnRfRxInterrupt
* ���� ��  ���߽����ж�
*************************************************************************************************/
static void awsnRfRxInterrupt(void)
{
  awsnRfPktHdr_t *pHdr;
  uint8 *pStatusWord;

  pHdr = (awsnRfPktHdr_t*)rxMpdu;

  //����жϱ�־����ֹ�µ�RX�����ж�
  halRfDisableRxInterrupt();

  //���������ж�
  halIntOn();

  //��ȡ���ճ���
  halRfReadRxBuf(&pHdr->packetLength,1);          //��ȡRxBuf��һ�ֽ����ݣ�MAC���ܳ���
  pHdr->packetLength &= AWSN_RF_PLD_LEN_MASK;     //�������bit��������Чֵ

  //��ȡ��������
  halRfReadRxBuf(&rxMpdu[1],pHdr->packetLength);
  #ifdef SENSOR
  //�������ڵ�
  if(pHdr->packetLength == PKT_LEN_ACK + MAC_LEN_OVERHEAD) {
    //�����Ӧ��֡
    rxi.pPayload = rxMpdu + MAC_LEN_HEAD + 1;
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ACK + 1;
    if((pStatusWord[1] & AWSN_RF_CRC_OK_BM)) {
      txState.rssi = rxi.pPayload[0];    //��ȡ�ɼ������ص�RSSI
      txState.lqi = rxi.pPayload[1];   //��ȡ�ɼ������ص�LQI
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ACK;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_ADD + MAC_LEN_OVERHEAD) {
    //��ַ����֡
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ADD;
    if((*pStatusWord & AWSN_RF_CRC_OK_BM)) {
      pConfig->panId = rxMpdu[MAC_LEN_HEAD];
      pConfig->myAddr = rxMpdu[MAC_LEN_HEAD+1];
      txState.ackReceived = TRUE;
      rxi.status = AWSN_ADD;
    }
  }
  else if(pHdr->packetLength == PKT_LEN_SYN + MAC_LEN_OVERHEAD) {
    //ͬ��֡
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_SYN;
    if(*pStatusWord & AWSN_RF_CRC_OK_BM)
    {
      pConfig->cycleTime = rxMpdu[MAC_LEN_HEAD];
      rxi.status = AWSN_SYN;
    }
  }
  #endif

  #ifdef GATEWAY
  //�ɼ����ڵ�
  if(pHdr->packetLength == PKT_LEN_ENTER + MAC_LEN_OVERHEAD) {
    //����֡
    pStatusWord = rxMpdu + MAC_LEN_HEAD + PKT_LEN_ENTER;
    if(*pStatusWord & AWSN_RF_CRC_OK_BM) {
      rxi.status = AWSN_ENTER;
    }
  }
  else {
    //����֡
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

  //RX�����ж�ʹ��
  halIntOff();
  halRfEnableRxInterrupt();

}



/*************************************************************************************************
* ���� ��  awsnRfInit
* ���� ��  RF��ʼ��
* ���� ��  pRfConfig - ���ýṹָ��
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

  //����Ƶ��
  halRfSetChannel(pConfig->channel);

  //д��̵�ַ
  halRfSetShortAddr(pConfig->myAddr);
  halRfSetPanId(pConfig->panId);

  //��ȡMAC��ַ
  awsnRfGetMacAddr();

  //���ý����ж�
  halRfRxInterruptConfig(awsnRfRxInterrupt);

  halIntOn();

  return SUCCESS;
}


/*************************************************************************************************
* ���� ��  awsnRfSensorSendPacket
* ���� ��  RF ������ģ�鷢������
* ���� ��  destAddr - Ŀ�ĵ�ַ
*         pPayload - ��������ָ��
*         length - ���س���
* ����ֵ��  SUCCESS or FAILED
*************************************************************************************************/
uint8 awsnRfSensorSendPacket(uint16 destAddr,uint8 *pPayload,uint8 length)
{
  uint8 mpduLength;
  uint8 status;

  //��RF����
  if(!txState.receiveOn) {
    halRfReceiveOn();
  }

  //��������
  length = min(length,AWSN_RF_LEN_MAX_PAYLOAD);

  //�ȴ����ͳɹ�
  halRfWaitTransceiverReady();

  //�ر�RF�����ж�
  halRfDisableRxInterrupt();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);

  //�򿪽����жϣ��ȴ�Ӧ��
  halRfEnableRxInterrupt();
  
  //�ȴ�Ӧ����Ϣ
  txState.ackReceived = FALSE;

  //��������
  if(halRfTransmit() != SUCCESS) {
    status = FAILED;
  }

  //��ʱ���ã�����12������ʱ�䣬Ӧ����Ĵ���ʱ�䣬��һЩ����ʱ��
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
* ���� ��  awsnRfGatewaySendPacket
* ���� ��  RF ����ģ�鷢������
* ���� ��  destAddr - Ŀ�ĵ�ַ
*         pPayload - ��������ָ��
*         length - ���س���
* ����ֵ��  SUCCESS or FAILED
*************************************************************************************************/
uint8 awsnRfGatewaySendPacket(uint16 destAddr,uint8 *pPayload,uint8 length)
{
  uint8 mpduLength;
  uint8 status;


  //��������
  length = min(length,AWSN_RF_LEN_MAX_PAYLOAD);

  //�ȴ����ͳɹ�
  halRfWaitTransceiverReady();

  mpduLength = awsnRfBuildMpdu(destAddr,pPayload,length);

  halRfWriteTxBuf(txMpdu,mpduLength);


  //��������
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
* ���� ��  awsnRfPacketIsReady
* ���� ��  ���ݽ����Ƿ����
* ����ֵ��
*************************************************************************************************/
uint8 awsnRfPacketIsReady(void)
{
  return rxi.isReady;
}


/*************************************************************************************************
* ���� ��  awsnRfRecive
* ���� ��  ���Ƹ������ݵ�buffer��
* ���� ��  pRxData - bufferָ��
*         len - ���س���
*         pRssi - �����ź�ǿ��
* ����ֵ��
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
* ���� ��  awsnRfRecieveOn
* ���� ��  ���մ�
*************************************************************************************************/
void awsnRfReceiveOn(void)
{
  txState.receiveOn = TRUE;
  halRfReceiveOn();
}


/*************************************************************************************************
* ���� ��  awsnRfRecieveOff
* ���� ��  ���չر�
*************************************************************************************************/
void awsnRfReceiveOff(void)
{
  txState.receiveOn = FALSE;
  halRfReceiveOff();
}


/*************************************************************************************************
* ���� ��  awsnRfMacAddr
* ���� ��  ��ȡMac��ַ
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
* ���� ��  awsnRfGetTxRssi
* ���� ��  ��ȡ�����ź�����
* ����ֵ�� �����յ��Ĵ�������RSSI
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
* ���� ��  awsnRfGetTxLqi
* ���� ��  ��ȡ�����ź�����
* ����ֵ�� �����յ��Ĵ�������LQI
*************************************************************************************************/
uint8 awsnRfGetTxLqi(void)
{
  return txState.lqi;
}


/*************************************************************************************************
* ���� ��  awsnRfSleep
* ���� ��  ����
* ���� ��  sleepTime - ˯��ʱ�䣬��λ10s
*************************************************************************************************/
void awsnRfSleep(uint8 sleepTime)
{
  uint16 sleepTotalTime = sleepTime * 10;
  uint8 i;

  if(sleepTime > 25)
  {
    for((i = sleepTotalTime / 255); i > 0; i--)
    {
      C51_RTC_EnterSleep(255);
      halMcuInit();       // ���³�ʼ��MCU
      halMcuWaitMs(5);    // �ȴ�5ms��ϵͳ�ȶ�
      halRfSetTxPower(pConfig->txPower);
      awsnRfReceiveOff();
    }
    C51_RTC_EnterSleep(sleepTotalTime % 255);
    halMcuInit();       // ���³�ʼ��MCU
    halMcuWaitMs(5);    // �ȴ�5ms��ϵͳ�ȶ�
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
  else if(sleepTime != 0){
    C51_RTC_EnterSleep(sleepTime * 10);   //˯��
    halMcuInit();       // ���³�ʼ��MCU
    halMcuWaitMs(5);    // �ȴ�5ms��ϵͳ�ȶ�
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
  else if(sleepTime == 0)
  {
    C51_RTC_EnterSleep(2);   //˯��
    halMcuInit();       // ���³�ʼ��MCU
    halMcuWaitMs(5);    // �ȴ�5ms��ϵͳ�ȶ�
    halRfSetTxPower(pConfig->txPower);
    awsnRfReceiveOff();
  }
}


/*************************************************************************************************
* ���� ��  awsnRfGetTxSrcAddr
* ���� ��  ��ȡ����Addr
* ����ֵ�� ���ͷ�srcaddr
*************************************************************************************************/
uint8 awsnRfGetTxSrcAddr(void)
{
  return rxi.srcAddr;
}

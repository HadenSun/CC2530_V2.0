/***********************************************************************************
  Filename:     awsn_rf.h

  Description:  AWSN(Agriculture Wireless Sensor Network) RF ��ͷ�ļ�

***********************************************************************************/

#ifndef AWSN_RF_H
#define AWSN_RF_H
/*********************************************ͷ�ļ�����******************************************/
#include "hal_types.h"
#include "hao_defs.h"
#include "delay.h"

/************************************************��������****************************************/
#define HAL_INFOP_IEEE_OSET 0xC


typedef enum {
  AWSN_ACK,
  AWSN_ENTER,
  AWSN_ADD,
  AWSN_SYN,
  AWSN_DATA,
  AWSN_NONE
} awsnRfRxState_t


typedef struct {
  uint8 macAddr[8];   //mac��ַ
  uint16 myAddr;      //�ڵ��ַ
  uint16 panId;       //�ڵ�Pan Id
  uint8 channel;      //RF ͨ����11-26֮�䣩
  uint8 txPower;      //���书��
  uint8 cycleTime;    //ͨ������
}awsnRfCfg_t


/************************************************��������****************************************/
uint8 awsnRfInit(awsnRfCfg_t* pRfConfig);                               //RF��ʼ��
uint8 awsnRfSendPacket(uint16 destAddr,uint8 *pPayload,uint8 length);   //RF��������
uint8 awsnRfPacketIsReady(void);                                        //���������Ƿ����
uint8 awsnRfRecieve(uint8 *pRxData,UINT8 len, int16* pRssi);            //���Ƹ������ݵ�buffer��

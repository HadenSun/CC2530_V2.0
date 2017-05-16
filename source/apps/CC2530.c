/***************************************************************************************
* �ɼ�����
*       ����ǰ���� GATEWAY
*       Ĭ���ϴ�ʱ�� 1min
*
* ��������
*       ����ǰ���� SENSOR
*       ��ѹ3v���� VCC_3V     ��ѹ5V���� VCC_5V
*       Ĭ���ϴ�ʱ�� 2s
***************************************************************************************/



/*********************************************ͷ�ļ�����******************************************/
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

/************************************************��������****************************************/
#define TX                  1       // ����ģʽ
#define RX                  0       // ����ģʽ

#define VCC3V               0       //3v����
#define VCC5V               1       //5v����

#define TX_ADDR             0x2509  // ���͵�ַ
#define RX_ADDR             0xBEEF  // ���յ�ַ

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  ����Ϊ5MHZ
#define PAN_ID              0x2007  // PAN ID


#define SEND_LENGTH         12      // ��������ÿ���ĳ���
#define CO2_SEND_LENGTH     3
#define RF_PKT_MAX_SIZE     30      // ���ݰ����ֵ

#define RSSI_H              -65     //RSSI����
#define RSSI_L              -90      //RSSI����


/************************************************ȫ�ֱ���****************************************/
static awsnRfCfg_t awsnRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // ���ջ���
static uint8    pTxData[SEND_LENGTH] = {0};  		// ��Ҫ���͵�����
static uint16   SendCnt = 0;                            // �������͵����ݰ���
static uint16   RecvCnt = 0;                            // �������յ����ݰ���
static uchar    Uart1_temp = 0;                         // ���ڽ�������


uint32 counter = 0;         //��ʱ���жϼ���
uchar  timer_flag = 0;      //ʱ���־
uint32 TIMER_COUNTER = (uint32)1000 * 240;         //��ʱ����ʱʱ�䣬��λms
u8 receive_trans_flag = 0;      //�յ���ת��
u8 CO2RecvFlag = 0;             //������̼���ڽ��ձ�־��0�����ڽ��գ�1���յ�0x0D��2���յ�0x0A��׼������
u8 CO2RecvCount = 0;            //������̼���ڽ��ռ���
u8 RF_TxCounter = 0;            //����ʧ�ܼ���
u8 UartRecvData[4] = {0};       //���ڽ������飬�ɼ�����
u8 UartRecvCounter = 0;         //���ڽ�������������ɼ�����

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


/************************************************��������****************************************/
static void Data_Init();                        //���ݳ�ʼ��
static u8 RF_SendPacket(void);                  // ���ͺ���
static void RF_RecvPacket(void);                  // ���պ���
static void RF_Initial(uint8 mode);               // RF��ʼ��
static void RST_System(void);                     // ����ϵͳ


/*************************************************************************************************
* ���� ��   Data_Init() => ��ʼ������
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
* ���� ��   RF_Initial() => ��ʼ��RFоƬ
* ���� ��   mode, =0,����ģʽ�� else,����ģʽ
*************************************************************************************************/
static void RF_Initial(uint8 mode)
{
    // ���õ�ַ
    if (RX == mode)     { awsnRfConfig.myAddr = RX_ADDR; }
    else                { awsnRfConfig.myAddr = TX_ADDR; }

    awsnRfConfig.panId = PAN_ID;           // ���ýڵ�PAN ID
    awsnRfConfig.channel = RF_CHANNEL;     // ���ýڵ��ŵ�
    awsnRfConfig.txPower = 1;
#ifdef SENSOR
    awsnRfConfig.cycleTime = 0;
#endif
#ifdef GATEWAY
    awsnRfConfig.cycleTime = 6;
    TIMER_COUNTER = (uint32)10000 * awsnRfConfig.cycleTime;
#endif

    if (awsnRfInit(&awsnRfConfig) == FAILED)      { HAL_ASSERT(FALSE); }

    if(RX == mode)    { halRfSetTxPower(2); }                     // �����������Ϊ4dbm
    else              { halRfSetTxPower(2); }

    if (RX == mode)     { awsnRfReceiveOn();  }
    else                { awsnRfReceiveOff(); }
}

/*************************************************************************************************
* ���� ��RST_System() => ��������ϵͳ������ϵͳ�����Ѻ�
*************************************************************************************************/
static void RST_System(void)
{
    halMcuInit();       // ���³�ʼ��MCU
    halMcuWaitMs(5);    // �ȴ�5ms��ϵͳ�ȶ�
    RF_Initial(TX);     // ���³�ʼ��RF
}

/*************************************************************************************************
* ���� : BSP_RF_SendPacket() => ���߷������ݺ���                            *
* ˵�� ��Sendbufferָ������͵����ݣ�length�������ݳ���                     *
*************************************************************************************************/
static u8 RF_SendPacket(void)
{
  u8 states = 1;
  // ����һ�����ݣ����ж��Ƿ��ͳɹ����յ�Ӧ��
  if(pointType == VCC3V)
  {
    if (!awsnRfSensorSendPacket(RX_ADDR, pTxData, SEND_LENGTH))
    {
        SendCnt++;
        halLedClear(1);         // LED��˸������ָʾ���ͳɹ������յ�Ӧ��
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
    if(RF_TxCounter++ > 3)
    {                                   //����ʧ��3�κ�Ӵ��书��
      if(awsnRfConfig.txPower != 2)
      {
        awsnRfConfig.txPower++;
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }
    else if(RF_TxCounter > 14)          //�ظ�����14�� 30s ��
    {
      states = 0;
      awsnRfConfig.cycleTime = 30;      //���ܲɼ���δ����ÿ 5min ��������һ��
    }
  }
  else{
    //�ɹ��յ�Ӧ���źź���ȡ�ź�����
    if(awsnRfGetTxRssi() > RSSI_H)      //����ź�����������ֵ
    {
      if(awsnRfConfig.txPower != 0)
      {
        awsnRfConfig.txPower--;       //���ͷ��书��
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }
    else if(awsnRfGetTxRssi() < RSSI_L)   //����ź�����С����ֵ
    {
      if(awsnRfConfig.txPower != 2)
      {
        awsnRfConfig.txPower++;         //���ӷ��书��
        halRfSetTxPower(awsnRfConfig.txPower);
      }
    }

    //�޸Ĳ�������
    awsnRfConfig.cycleTime = awsnRfGetTxLqi();
  }

  if(states)
  {
    C51_RTC_EnterSleep(2);       // ϵͳ����ģʽPM2���͹��ģ�2s��RTC����
  }
  else
    awsnRfSleep(awsnRfConfig.cycleTime);    //ϵͳ����ģʽPM2���͹��ģ�˯��ʱ���������·�

  return states;
}

/*************************************************************************************************
* ���� ��RF_RecvPacket() => �������ݽ��մ���
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    u16 srcAddr = 0;
    int8 rssi = 0;

    if(!awsnRfPacketIsReady())    // ���ģ���Ƿ��Ѿ����Խ�����һ������
      return;

    
    length=awsnRfReceive(pRxData, RF_PKT_MAX_SIZE, &rssi);
    // ���յ������ݸ��Ƶ�pRxData��
    if (length > 0)
    {
        // �жϽ��������Ƿ���ȷ
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


          // ��˸LED��ָʾ�յ���ȷ����
          halLedClear(1);
          halMcuWaitMs(500);
          halLedSet(1);
        }
    }
}




/*************************************************************************************************
* ���� ��Timer3_Init() => Time3��ʼ��
* ˵�� ��ÿ���ж�0.1ms
*************************************************************************************************/
void Timer3_Init(void)
{
  T3CC0 = 0xFA;     //����125��1msһ���ж�
  T3CCTL0 |= 0x04;
  T3CTL = 0xFE;     //32Mʱ��Ƶ�ʣ�128��Ƶ���ж�ʹ�ܣ������ʱ��������ģʽ
  T3IE = 1;
  EA = 1;
}

/*************************************************************************************************
* ���� : main() => ���������������

*************************************************************************************************/
void main(void)
{
    u8 error = 0;

    //SHT1x
    u8 stateRegister;  //�û��Ĵ���
    u16 T,RH;         //��ʪ��

    //BMP280
    BMP280_U32_t rst;

    //co2
    u8 s100Rec[10];
    u8 uart1RxTemp = 0;
    u16 co2PPM = 0;



    halBoardInit();                                         // ��ʼ��Χ�豸

    IIC_Init();                                             // IIC��ʼ��

    if(appMode == RX) Uart1_Init(UART_BAUD_115200,1);                    // Uart1��ʼ��,115200
    else              Uart1_Init(UART_BAUD_38400,0);


    if (appMode == TX)              { C51_RTC_Initial(); }  // ��ʼ��RTC�����ͷ���Ҫ˯��
    else                            { Timer3_Init(); }

    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // ��Ӳ��������rf���г�ʼ��

    RF_Initial(appMode);                                    // ��ʼ��RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO��⣬�ж�IO�Ƿ��ж̽ӣ���·

    //C51_GPIO_OffDetective();                                // �����޹�IOΪ���룬���͹���


    //�����ʼ��
    if(appMode == TX && pointType == VCC3V)
    {
      halMcuWaitMs(3000);

      //SHT1x��ʼ��
      SHT1x_ReConnect();

      //BMP280��ʼ��
      BMP280_Init();
    }


    while (1)
    {
      if(appMode == RX)
      {
        RF_RecvPacket();        //RF����
        //if(timer_flag && (!receive_trans_flag))     //����ʹ��
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
          error |= MAX_ReadLightData(pTxData+1,pTxData+2);     //����ǿ������
          pTxData[0] = 0x10 + error;
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);

          //SHT1x
          error = 0;
          error |= SHT1x_ReadTempResult(&T);           //��ȡ�¶���Ϣ
          pTxData[4] = (T >> 8) & 0xFF;
          pTxData[5] = T & 0xFF;
          pTxData[3] = 0x20 + error;
          //Uart1_SendByte(error);
          //Uart1_SendByte(pTxData[3]);
          //Uart1_SendByte(pTxData[4]);
          //Uart1_SendByte(checksum);

          error = 0;
          error |= SHT1x_ReadRhResult(&RH);      //��ȡʪ����Ϣ
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
            URX1IF = 0;       //���жϱ�־
            uart1RxTemp = U1DBUF;

            if(uart1RxTemp == 0x0a)
            {
              if(CO2RecvFlag == 1)            //�Ѿ��յ�0x0d
              {
                if(CO2RecvCount == 10)        //�Ѿ��յ�10����Ч����
                {
                  co2PPM = 0;
                  if(s100Rec[0] >= '0' && s100Rec[0] <= '9')      //��λ��Ӧ��û��
                    co2PPM += (s100Rec[0] - '0') * 10000;
                  if(s100Rec[1] >= '0' && s100Rec[1] <= '9')      //ǧλ
                    co2PPM += (s100Rec[1] - '0') * 1000;
                  if(s100Rec[2] >= '0' && s100Rec[2] <= '9')      //��λ
                    co2PPM += (s100Rec[2] - '0') * 100;
                  if(s100Rec[3] >= '0' && s100Rec[3] <= '9')      //ʮλ
                    co2PPM += (s100Rec[3] - '0') * 10;
                  if(s100Rec[4] >= '0' && s100Rec[4] <= '9')      //��λ
                    co2PPM += s100Rec[4] - '0';

                  pTxData[0] = 0;
                  pTxData[1] = co2PPM>>8;
                  pTxData[2] = co2PPM & 0xFF;

                  CO2RecvFlag = 2;      //׼������
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
            else if(uart1RxTemp == 0x0D)      //�յ�0x0d
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
            else      //�յ����ֻ�ո�
            {
              if(CO2RecvCount < 9)
                s100Rec[CO2RecvCount++] = uart1RxTemp;
              else{
                CO2RecvCount = 0;
                CO2RecvFlag = 0;
              }
            }
          }

          if(CO2RecvFlag == 2){       //׼������
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

    halBoardInit();                                         // ��ʼ��Χ�豸

    IIC_Init();                                             // IIC��ʼ��

    if(appMode == RX) Uart1_Init(UART_BAUD_115200,1);                    // Uart1��ʼ��,115200
    else              Uart1_Init(UART_BAUD_38400,0);


    if (appMode == TX)              { C51_RTC_Initial(); }  // ��ʼ��RTC�����ͷ���Ҫ˯��
    else                            { Timer3_Init(); }

    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // ��Ӳ��������rf���г�ʼ��

    RF_Initial(appMode);                                    // ��ʼ��RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO��⣬�ж�IO�Ƿ��ж̽ӣ���·

    //C51_GPIO_OffDetective();                                // �����޹�IOΪ���룬���͹���

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
* ���� : UART0_ISR() => Uart1�����ж�
* ˵�� ��
****************************************************************/
#pragma vector = URX1_VECTOR
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//���жϱ�־
	Uart1_temp = U1DBUF;
        if(Uart1_temp == 0xA5)                  //��ʼλ
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
* ���� : T3_ISR() => Timer3 �ж�
* ˵�� �����ɼ���ʹ�ã���������
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

/***************************************************************************************
* ��Ҫע��Ĳ���ֵ
* ȫ�ֱ�����
*     pointType - ����5V/3V
*     appMode - ������ɫ ��/��
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

#define TX_ADDR             0x2500  // ���͵�ַ
#define RX_ADDR             0xBEEF  // ���յ�ַ

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  ����Ϊ5MHZ
#define PAN_ID              0x2007  // PAN ID


#define SEND_LENGTH         30      // ��������ÿ���ĳ���
#define CO2_SEND_LENGTH     2
#define RF_PKT_MAX_SIZE     30      // ���ݰ����ֵ


/************************************************ȫ�ֱ���****************************************/
static awsnRfCfg_t awsnRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // ���ջ���
static uint8    pTxData[SEND_LENGTH] = { 0x00 };  		 	// ��Ҫ���͵�����
static uint16   SendCnt = 0;                            // �������͵����ݰ���
static uint16   RecvCnt = 0;                            // �������յ����ݰ���
static uchar    Uart1_temp = 0;                         // ���ڽ�������

static uchar    TransRst[SEND_LENGTH];                  // ��Ҫ���͵�����

uint32 counter = 0;         //��ʱ���жϼ���
uchar  timer_flag = 0;      //ʱ���־
uint32 TIMER_COUNTER = (uint32)1000 * 240;         //��ʱ����ʱʱ�䣬��λms
u8 receive_trans_flag = 0;      //�յ���ת��
u8 CO2RecvFlag = 0;             //������̼���ڽ��ձ�־��0�����ڽ��գ�1���յ�0x0D��2���յ�0x0A��׼������
u8 CO2RecvCount = 0;            //������̼���ڽ��ռ���
u8 RF_TxCounter = 0;            //����ʧ�ܼ���


uint8 pointType = VCC3V;
uint8 appMode = RX;


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

    if (awsnRfInit(&awsnRfConfig) == FAILED)      { HAL_ASSERT(FALSE); }

//    halRfSetTxPower(1);                     // �����������Ϊ4dbm

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
      if (!basicRfSendPacket(RX_ADDR, TransRst, SEND_LENGTH))
      {
          SendCnt++;
          halLedClear(1);         // LED��˸������ָʾ���ͳɹ������յ�Ӧ��
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
      C51_RTC_EnterSleep(2);       // ϵͳ����ģʽPM2���͹��ģ�2s��RTC����
    }
    else
      C51_RTC_EnterSleep(240);
    RST_System();               // ���³�ʼ��ϵͳ
    return states;
}

/*************************************************************************************************
* ���� ��RF_RecvPacket() => �������ݽ��մ���
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    u16 srcAddr = 0;

    if(!basicRfPacketIsReady())    // ���ģ���Ƿ��Ѿ����Խ�����һ������
      return;
    else
      basicRfPacketIsReadyIn(FALSE);

    // ���յ������ݸ��Ƶ�pRxData��
    if ((length=basicRfReceive(pRxData, RF_PKT_MAX_SIZE, NULL)) > 0)
    {
        // �жϽ��������Ƿ���ȷ
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
            //if(receive_trans_flag)            //����ʹ��
            //{
            //  Uart1_SendData_Test(srcAddr);
            //  Uart1_SendByte('\r');
            //  Uart1_SendByte('\n');
            //}
            // ��˸LED��ָʾ�յ���ȷ����
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
* ˵�� �����ͷ���ÿ2s��RTC����һ�Σ���������һ�����ݣ�ÿ�����ݳ���Ϊ5���ֽڣ�"123/r/n",��ɫLED
         ��˸�������ͳɹ��������յ���ȷ��Ӧ�����ݡ�
         ���շ�: ������յ����ݳ���Ϊ5���ֽڣ���ǰ����Ϊ��123�������ݽ�����ȷ��ͬʱ��ɫLED��˸һ��
         ָʾ�յ���ȷ���ݡ�
         ע�⣺appMode = TX�����ͳ��򣩣� =RX�����ճ���
*************************************************************************************************/
void main(void)
{
    u8 error = 0;
    /*
    //SHT2x
    u8 userRegister;  //�û��Ĵ���
    nt16 sT;          //�¶�
    nt16 sRH;         //ʪ��
    */
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


    if(appMode == TX && pointType == VCC3V)
    {
      halMcuWaitMs(3000);
      /*
      //OPT3001��ʼ��
      error = 0;
      error = OPT3001_WriteRegister(OPT3001_REG_ADD_COF,0xCE10);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      */

      /*
      //SHT2x��ʼ��
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

      //SHT1x��ʼ��
      SHT1x_ReConnect();

      /*
      //BH1750��ʼ��
      error = 0;
      error |= BH1750_WriteCommand(BH1750_MODE_CH);
      Uart1_SendString("error:",6);
      Uart1_SendByte('0'+error);
      Uart1_SendByte('\r');
      Uart1_SendByte('\n');
      */

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

          /*
          //OPT3001
          error = 0;
          error |= OPT3001_ReadOriginalData(TransRst+1,TransRst+2);   //����ǿ������
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);
          */

          /*
          //bh1750
          //error = 0;
          error |= BH1750_ReadOriginalData(TransRst+1,TransRst+2);    //����ǿ������
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);
          */

          //MAX44009
          //error = 0;
          error |= MAX_ReadLightData(TransRst+1,TransRst+2);     //����ǿ������
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[1]);
          //Uart1_SendByte(TransRst[2]);


          /*
          //SHT2X
          //error = 0;
          //error |= SHT2x_SoftReset();
          //Uart1_SendByte(error);
          //error = 0;
          error |= SHT2x_ReadUserRegister(&userRegister);           //��ȡ�û��Ĵ���
          if( (userRegister & SHT2x_EOB_MASK) == SHT2x_EOB_ON )     //END OF BATTERYΪ1����ص�ѹ����
            error |= BATTERY_ALERT;
          Uart1_SendByte(error);
          //Uart1_SendByte(userRegister);
          //error = 0;
          //userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
          //error |= SHT2x_WriteUserRegister(&userRegister);
          //Uart1_SendByte(error);

          //error = 0;
          error |= SHT2x_MeasurePoll(TEMP,&sT);             //��ȡ�¶���Ϣ
          Uart1_SendByte(error);
          Uart1_SendByte(sT.s16.u8H);
          Uart1_SendByte(sT.s16.u8L);
          error = 0;
          error |= SHT2x_MeasurePoll(HUMIDITY,&sRH);        //��ȡʪ����Ϣ
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
          error |= SHT1x_ReadTempResult(&T);           //��ȡ�¶���Ϣ
          TransRst[3] = (T >> 8) & 0xFF;
          TransRst[4] = T & 0xFF;
          //Uart1_SendByte(error);
          //Uart1_SendByte(TransRst[3]);
          //Uart1_SendByte(TransRst[4]);
          //Uart1_SendByte(checksum);

          //error = 0;
          error |= SHT1x_ReadRhResult(&RH);      //��ȡʪ����Ϣ
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

                  TransRst[0] = co2PPM>>8;
                  TransRst[1] = co2PPM & 0xFF;

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


/****************************************************************
* ���� : UART0_ISR() => Uart1�����ж�
* ˵�� ��
****************************************************************/
#pragma vector = URX1_VECTOR
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//���жϱ�־
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

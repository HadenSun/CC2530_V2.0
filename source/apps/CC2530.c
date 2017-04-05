/*********************************************ͷ�ļ�����******************************************/
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

/************************************************��������****************************************/
#define TX                  1       // ����ģʽ
#define RX                  0       // ����ģʽ

#define TX_ADDR             0x2520  // ���͵�ַ
#define RX_ADDR             0xBEEF  // ���յ�ַ

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  ����Ϊ5MHZ
#define PAN_ID              0x2007  // PAN ID

#define SEND_LENGTH         7       // ��������ÿ���ĳ���
#define RF_PKT_MAX_SIZE     30      // ���ݰ����ֵ

/************************************************ȫ�ֱ���****************************************/
static basicRfCfg_t basicRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // ���ջ���
static uint8    pTxData[SEND_LENGTH] = { "123\r\n" };   // ��Ҫ���͵�����
static uint16   SendCnt = 0;                            // �������͵����ݰ���
static uint16   RecvCnt = 0;                            // �������յ����ݰ���
static uchar    Uart1_temp = 0;                         // ���ڽ�������

static uchar    TransRst[SEND_LENGTH];                  // ��Ҫ���͵�����

/************************************************��������****************************************/
static void RF_SendPacket(void);                  // ���ͺ���
static void RF_RecvPacket(void);                  // ���պ���
static void RF_Initial(uint8 mode);               // RF��ʼ��
static void RST_System(void);                     // ����ϵͳ
static void Uart1_Init(void);                     // Uart1��ʼ��
static void Uart1_SendString(char *Date,int len); // Uart1�����ַ���
static void Uart1_SendByte(uchar n);              // Uart1�����ֽ�

/*************************************************************************************************
* ���� ��   RF_Initial() => ��ʼ��RFоƬ
* ���� ��   mode, =0,����ģʽ�� else,����ģʽ
*************************************************************************************************/
static void RF_Initial(uint8 mode)
{
    // ���õ�ַ
	if (RX == mode)     { basicRfConfig.myAddr = RX_ADDR; }
	else                { basicRfConfig.myAddr = TX_ADDR; }

    basicRfConfig.panId = PAN_ID;           // ���ýڵ�PAN ID
    basicRfConfig.channel = RF_CHANNEL;     // ���ýڵ��ŵ�
    basicRfConfig.ackRequest = TRUE;        // Ӧ������

    if (basicRfInit(&basicRfConfig) == FAILED)      { HAL_ASSERT(FALSE); }

//    halRfSetTxPower(1);                     // �����������Ϊ4dbm

    if (RX == mode)     { basicRfReceiveOn();  }
    else                { basicRfReceiveOff(); }
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
static void RF_SendPacket(void)
{
    // ����һ�����ݣ����ж��Ƿ��ͳɹ����յ�Ӧ��
    if (!basicRfSendPacket(RX_ADDR, TransRst, SEND_LENGTH));
    {
        SendCnt++;
        halLedClear(1);         // LED��˸������ָʾ���ͳɹ������յ�Ӧ��
    }
    halMcuWaitMs(500);
    halLedSet(1);
    C51_RTC_EnterSleep();       // ϵͳ����ģʽPM2���͹��ģ�2s��RTC����
    RST_System();               // ���³�ʼ��ϵͳ
}

/*************************************************************************************************
* ���� ��RF_RecvPacket() => �������ݽ��մ���
*************************************************************************************************/
static void RF_RecvPacket(void)
{
    uint8 length = 0;
    u16 srcAddr = 0;

    while (!basicRfPacketIsReady());    // ���ģ���Ƿ��Ѿ����Խ�����һ������

    // ���յ������ݸ��Ƶ�pRxData��
    if ((length=basicRfReceive(pRxData, RF_PKT_MAX_SIZE, NULL)) > 0)
    {
        // �жϽ��������Ƿ���ȷ
        if ((SEND_LENGTH==length))
        {
            RecvCnt++;
            srcAddr = basicRfGetTxSrcAddr();
            Uart1_SendByte((srcAddr>>8) & 0xFF);
            Uart1_SendByte(srcAddr & 0xFF);
            for(length = 0;length < SEND_LENGTH;length++)
            {
              Uart1_SendByte(pRxData[length]);
            }
            Uart1_SendByte('\r');
            Uart1_SendByte('\n');
            // ��˸LED��ָʾ�յ���ȷ����
            halLedClear(1);
            halMcuWaitMs(500);
            halLedSet(1);
        }
    }
}


/*************************************************************************************************
* ���� ��Uart1_Init() => Uart1��ʼ��
* ˵�� ��ʹ��Uart1Ĭ�����ţ�P0.4-TX��P0.5-RX
*************************************************************************************************/
void Uart1_Init(void)
{

    PERCFG = 0x00;                //UART1ʹ��Ĭ������ P0.4 P0.5
    P0SEL = 0x30;                 //P0.4 P0.5����
    P2DIR |= 0x40;                //UART1����

    U1CSR |= 0x80;                //ѡ��UARTģʽ
    U1GCR |= 11;
    U1BAUD |= 216;                //������115200
    UTX1IF = 0;                   //����жϱ�־
    U1CSR |= 0x40;                //UART����ʹ��
    IEN0 |= 0x88;                 //���ж�ʹ�ܣ�UART1 RX�ж�ʹ��
}

/*************************************************************************************************
* ���� ��Uart1_SendString(char *Date,int len) => UART1�����ַ���
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
* ���� ��Uart1_SendByte(uchar n) => UART1�����ֽ�
*************************************************************************************************/
void Uart1_SendByte(uchar n)
{
  U1DBUF = n;
  while(!UTX1IF);
  UTX1IF = 0;
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
    u8 checksum;      //У����Ϣ
   
    uint8 appMode = RX;

    halBoardInit();                                         // ��ʼ��Χ�豸

    IIC_Init();                                             // IIC��ʼ��

    Uart1_Init();                                           // Uart1��ʼ��

    if (appMode == TX)              { C51_RTC_Initial(); }  // ��ʼ��RTC�����ͷ���Ҫ˯��

    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // ��Ӳ��������rf���г�ʼ��

    RF_Initial(appMode);                                    // ��ʼ��RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO��⣬�ж�IO�Ƿ��ж̽ӣ���·

    //C51_GPIO_OffDetective();                                // �����޹�IOΪ���룬���͹���
    
    if(appMode == TX)
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
    }
    
    
    while (1)
    {
      if(appMode == RX)
      {
        RF_RecvPacket();        //RF����
      }
      else if(appMode == TX)
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
        error |= MAX_ReadOriginalData(TransRst+1,TransRst+2);     //����ǿ������
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
        error |= SHT1x_ReadResult(&T,&checksum,SHT1x_TEMP);           //��ȡ�¶���Ϣ
        TransRst[3] = (T >> 8) & 0xFF;
        TransRst[4] = T & 0xFF;
        //Uart1_SendByte(error);
        //Uart1_SendByte(TransRst[3]);
        //Uart1_SendByte(TransRst[4]);
        //Uart1_SendByte(checksum);
        
        //error = 0;
        error |= SHT1x_ReadResult(&RH,&checksum,SHT1x_HUMIDITY);      //��ȡʪ����Ϣ
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

        
        
        TransRst[0] = error;

        RF_SendPacket();
      }


    }
}


/****************************************************************
* ���� : UART0_ISR() => Uart1�����ж�
* ˵�� ��
****************************************************************/
#pragma vector = URX0_VECTOR
 __interrupt void UART0_ISR(void)
 {
 	URX0IF = 0;				//���жϱ�־
	Uart1_temp = U0DBUF;                          
 }
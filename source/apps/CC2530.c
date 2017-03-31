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

/************************************************��������****************************************/
#define TX                  1       // ����ģʽ
#define RX                  0       // ����ģʽ

#define TX_ADDR             0x2520  // ���͵�ַ                 
#define RX_ADDR             0xBEEF  // ���յ�ַ

#define RF_CHANNEL          25      // 2.4 GHz RF channel 11 - 26  ����Ϊ5MHZ
#define PAN_ID              0x2007  // PAN ID

#define SEND_LENGTH         5       // ��������ÿ���ĳ���
#define RF_PKT_MAX_SIZE     30      // ���ݰ����ֵ

/************************************************ȫ�ֱ���****************************************/
static basicRfCfg_t basicRfConfig;
static uint8    pRxData[RF_PKT_MAX_SIZE];               // ���ջ���
static uint8    pTxData[SEND_LENGTH] = { "123\r\n" };   // ��Ҫ���͵�����
static uint16   SendCnt = 0;                            // �������͵����ݰ���
static uint16   RecvCnt = 0;                            // �������յ����ݰ���

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
    if (!basicRfSendPacket(RX_ADDR, pTxData, SEND_LENGTH));
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
    
    while (!basicRfPacketIsReady());    // ���ģ���Ƿ��Ѿ����Խ�����һ������

    // ���յ������ݸ��Ƶ�pRxData��
    if ((length=basicRfReceive(pRxData, RF_PKT_MAX_SIZE, NULL)) > 0) 
    {
        // �жϽ��������Ƿ���ȷ
        if ((SEND_LENGTH==length) && (pRxData[0]=='1')\
        && (pRxData[1]=='2') && (pRxData[2]=='3'))
        {
            RecvCnt++;
            
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
    float rst;
    float hum = 0;
    float tem = 0;
    u8 error = 0;
    u8 userRegister;
    nt16 sT;
    nt16 sRH;
    uint8 appMode = TX;
    
    halBoardInit();                                         // ��ʼ��Χ�豸
    
    IIC_Init();                                             // IIC��ʼ��
    
    Uart1_Init();                                            // Uart1��ʼ��

    if (appMode == TX)              { C51_RTC_Initial(); }  // ��ʼ��RTC
    
    if (halRfInit() == FAILED)      { HAL_ASSERT(FALSE); }  // ��Ӳ��������rf���г�ʼ��
       
    RF_Initial(appMode);                                    // ��ʼ��RF

    if (!C51_GPIO_ioDetective())    { halLedClear(2); }     // IO��⣬�ж�IO�Ƿ��ж̽ӣ���·
    
    //C51_GPIO_OffDetective();                                // �����޹�IOΪ���룬���͹���
    
    
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
        //if      (appMode == RX)     { RF_RecvPacket(); }    // ����ģ��
        //else if (appMode == TX)     { RF_SendPacket(); }    // ����ģ��
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


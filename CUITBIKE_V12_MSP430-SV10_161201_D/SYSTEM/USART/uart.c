
#include "includes.h"

//*************************���¶������������͵��������**************************
#define FRAMEBUF_SIZE 22					//���֡����
#define IDLELINE_TIME 10					//��·�����ж�ʱ��
unsigned char FrameBuff[FRAMEBUF_SIZE];	//����֡������
unsigned char UART_RcvCnt = 0;				//���ռ���
//******************************************************************************

//static unsigned char RecevieBuf[24] = {0};	//900HP���յ���ָ��洢�������顣����롣
extern volatile unsigned int FlagUSCIA1;
extern unsigned char SamplingPeriod[2], SamplingPoint[2], SamplingFrequency[2];;
extern uchar TerminalNum[6], SensorNum[4];
extern uchar a[2];
extern float Temperature , BatteryVoltage;
extern float maxz, maxiz;
extern float ReplyZgDatas[512];	//�洢�����ź�;
typedef union FLOAT
{
  	float f_data;
  	uchar hex_float[4];
}uf;
uf hex_float4;

/******************************************************************************
** �������ƣ�void Usart_GPIO_Init()  
** ��    �ܣ�����IO�ڳ�ʼ��
** �޸���־��
*******************************************************************************/
void Usart_GPIO_Init(void) 
{
  	USART_TX_232_SEL;
  	USART_RX_232_SEL;
}
/******************************************************************************
** �������ƣ�void Usart_Wire_Temp_Init_9600(void)
** ��    �ܣ�����A0��ʼ�� ,���ղ�ͬҪ��Ĳ����ʽ��г�ʼ��
** �޸���־��
*******************************************************************************/
void Usart_Wire_Temp_Init_9600(void)
{
//    UCA1CTL1 &=~UCSWRST;              // **Put state machine in reset**
//    UCA1CTL1 &=~UCSSEL__SMCLK;        // SMCLK

    UCA1CTL1 |= UCSWRST;              // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL__SMCLK;        // SMCLK

    UCA1BR0 = 0X23;                   //baud ���òο� user's guide Page 954
    UCA1BR1 = 0x08;					// 20MHz/(8*256+37) = 9600

    UCA1MCTL =0X00;                  // Modulation
    UCA1CTL1 &= ~UCSWRST;            // **Initialize USCI state machine**
    return;
}
/*****************************************************************************
** �������ƣ�Send_232_data(uchar *tdata,uint data_length);
** ��    �ܣ�232���ݷ��� UART_A1
** �޸���־��
*****************************************************************************/
void Send_232_data(unsigned char *tdata, unsigned int data_length)
{
//   while (!(UCA1IFG & UCTXIFG));
   for(unsigned char s = 0; s < data_length; s++)
   {
    	while(!(UCA1IFG & UCTXIFG));	// �ж��Ƿ������
    	Ext_Wdog_Feed();
		UCA1TXBUF = *(tdata + s);
   }
   while (!(UCA1IFG & UCTXIFG));		// �ж��Ƿ������  
}
/*************************************************************************
** Name:		void Receive_232_data(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	��������ָ����ݲ�ָͬ��ز�ͬ��Ϣ��
**				���"G:\HUIYUAN COMMUNICATION\01΢����\�ĵ�\΢���񶯴�����ͨѶ��Լ.doc"		
** Version:	
*************************************************************************/
void Receive_232_data(void)
{
	UCA1CTL1 &= ~UCSWRST;                                        // ʹ�ܴ��ڹ���
    UCA1IE |= UCRXIE;                                            // ʹ�ܽ����ж�
}
/*************************************************************************
** Name:		__interrupt void USCI_A1_ISR(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	Interrput of UCRXIE
** Version:		
*************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	switch(__even_in_range(UCA1IV,4))
  	{	
  		case 0:
		  	break;                             // Vector 0 - no interrupt
  		case 2:                                // Vector 2 - RXIFG
    		TB0R = 0;
			FlagUSCIA1 = 1;
		  	if(UART_RcvCnt < FRAMEBUF_SIZE)
			{
				FrameBuff[UART_RcvCnt] = UCA1RXBUF;	//���յ����ֽڴ��뻺��
				UART_RcvCnt++;						//����+1
			}
			else
			{
				UCA1IE &= ~UCRXIE;				//�ﵽ��������С�󣬽�ֹ���ڽ����ж�
				UART_RcvCnt = 0;
			}
    		break;
  		case 4:
		  	break;                             // Vector 4 - TXIFG
  		default: 
		  	break;
  	}
}
/*************************************************************************
** Name:		void RecevieBufReading(unsigned char *RxBuf)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	������	����
				C0H		���ô���������
				C1H		���ò�������
				C2H		�����񶯷�ֵ��Ƶ��
				C3H		�����񶯲����ź�
				C4H		�����ص�ѹ���¶�
	�жϹ���	��	�����жϽ��յ�������У�����Ƿ���ȷ��
				���жϽ��յ��������д����������Ƿ���ȷ��
				����OK��
				��ʼ���������޸�ϵͳ���á�
** Version:	
*************************************************************************/
void RecevieBufReading(unsigned char *RxBuf)
{
  	unsigned char i = 0, FlagRxCheck = 0, FlagSensorNumCheck = 0;
	unsigned char RxSensorNum[4] = {0};
//*************************У����������Ƿ���ȷ**********************************
	FlagRxCheck = RxCheck(RxBuf);
	for(i = 0; i < 4; i++)
	{
		RxSensorNum[i] = RxBuf[i + 7];		//�洢���������еĴ���������
	}
	if(FlagRxCheck == 0)
	{
		FlagSensorNumCheck = memcmp(RxSensorNum, SensorNum, 4);	//У�鴫���������Ƿ�һ������ȷ���0
		if(FlagSensorNumCheck == 0)
		{
		  	ReplyTrue();
			switch(RxBuf[11])	//���ݿ������ж�ָ��
			{
		  		case 0xC0:
				  	for(i = 0; i < 4; i++)
					{
						SensorNum[i] = RxBuf[i + 14];
					}
//******************************�޸Ĵ��������룬���浽EEPROM*********************					
					P2OUT &= ~BIT5;
					EepromWritePage(0x01, 	SensorNum, 	4);	
					P2OUT |= BIT5;
				  	break;
				case 0xC1:	
//						SamplingPeriod = RxBuf[14] * 256 + RxBuf[15];		//�õ��������ڣ���λ����
//						SamplingPoint = RxBuf[16] * 256 + RxBuf[17];		//�õ�������������λ��
//						SamplingFrequency = RxBuf[18] * 256 + RxBuf[19];	//�õ�����Ƶ��,��λHz						
						for(i = 0; i < 2; i++)
						{
							SamplingPeriod[i] = RxBuf[i + 14];
						}
						for(i = 0; i < 2; i++)
						{
							SamplingPoint[i] = RxBuf[i + 16];
						}
						for(i = 0; i < 2; i++)
						{
							SamplingFrequency[i] = RxBuf[i + 18];
						}
//******************************�޸Ĳ������������浽EEPROM***********************					
						P2OUT &= ~BIT5;
						EepromWritePage(0x02, 	SamplingPeriod, 	2);	
						EepromWritePage(0x03, 	SamplingPoint, 		2);	
						EepromWritePage(0x04, 	SamplingFrequency, 	2);
						P2OUT |= BIT5;
						break;
				case 0xC2:
				  	ReplyAmplitudeAndFrequency();
				  	break;
				case 0xC3:
				  	ReplyWaveform();
			  		break;
				case 0xC4:
				  	ReplyVoltageAndTemperature();
			  		break;
				default:
			  		break;
			}
		}
		else
		{
			ReplyFalse();
		}
	}
}
/*************************************************************************
** Name:		void ReplyAmplitudeAndFrequency(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	�ظ��񶯷�ֵ����Ƶ�ʡ�
** Version:	
*************************************************************************/
void ReplyAmplitudeAndFrequency(void)
{
	unsigned char ReplyUsart[26] = {0x68};
	unsigned char Temp[4] = {0};
	unsigned char ReplyTemp[7] = {0xD0,0x00,0x0a,0x00,0x00,0x00,0x00};
	unsigned char i = 0;
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//����������
	memcpy(&ReplyUsart[11], ReplyTemp, 7);		//
	
	hex_float4.f_data = maxiz;		//�õ��񶯷�ֵ
	for(i = 2; i < 4; i++)			//hex_float4.hex_float��4�ֽڣ���VibrationAmplitude��2�ֽڣ�
	{
			Temp[i - 2] = hex_float4.hex_float[5 - i];
	}
	memcpy(&ReplyUsart[18], Temp, 2);
	
	hex_float4.f_data = maxz;		//�õ���Ƶ��
	for(i = 0; i < 4; i++)		
	{
			Temp[3 - i] = hex_float4.hex_float[i];
	}
	memcpy(&ReplyUsart[20], Temp, 4);
	
	a[0] = check_bit(ReplyUsart, 24);
	memcpy(&ReplyUsart[24], a, 2);
	Ext_Wdog_Feed();

  	Send_232_data(ReplyUsart, 26);
		
 	Ext_Wdog_Feed();
}
/*************************************************************************
** Name:		void ReplyWaveform(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	�ظ��񶯲��Ρ������ٶȵ�ԭʼ����ֵ����������64�ֽ�
** Version:	
*************************************************************************/
void ReplyWaveform(void)
{
  	unsigned char ReplyUsart[80] = {0x68};
	unsigned char ReplyTemp[3] = {0xD1,0x00,0x40};
	unsigned i = 0; 
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//����������
	memcpy(&ReplyUsart[11], ReplyTemp, 3);		//
//*******���صĲ������ݸ�ʽ�硰D0H����ʵ�ʼ���ʱӦ��Ϊ��D0 00 00 00������*********	
	for(i = 0; i < 64; i++)						
	{
		hex_float4.f_data = ReplyZgDatas[i];
		memcpy(&ReplyUsart[14 + i], &hex_float4.hex_float[3], 1);	//�������ݱ���64�ֽ�
	}
//*****************************************************************************
	a[0] = check_bit(ReplyUsart, 78);
	memcpy(&ReplyUsart[78], a, 2);
	Ext_Wdog_Feed();

  	Send_232_data(ReplyUsart, 80);
		
 	Ext_Wdog_Feed();
}
/*************************************************************************
** Name:		void ReplyVoltageAndTemperature(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.28
** Function:	�ظ���ص�ѹ���¶����ݡ�
** Version:	
*************************************************************************/
void ReplyVoltageAndTemperature(void)
{
	unsigned char ReplyUsart[24] = {0x68};
	unsigned char ReplyTemp[3] = {0xD2,0x00,0x08};//0x40 ��> 64����������
	unsigned char Temp[4] = {0}, i = 0;
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//����������
	memcpy(&ReplyUsart[11], ReplyTemp, 3);	//
	
	hex_float4.f_data = BatteryVoltage;	//�õ���ص�ѹ
	for(i = 0; i < 4; i++)		
	{
		Temp[3 - i] = hex_float4.hex_float[i];
	}
	memcpy(&ReplyUsart[14], Temp, 4);
	hex_float4.f_data = Temperature;		//�õ��¶�
	for(i = 0; i < 4; i++)		
	{
		Temp[3 - i] = hex_float4.hex_float[i];
	}
	memcpy(&ReplyUsart[18], Temp, 4);
	
	a[0] = check_bit(ReplyUsart, 19);
	memcpy(&ReplyUsart[22], a, 2);
	Ext_Wdog_Feed();
	
  	Send_232_data(ReplyUsart, 24);
	
 	Ext_Wdog_Feed();
}
/*******************************************************************************
** Name:		void ReplyTrue(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	��ִ����ɽ���ָ��󣬻ظ�0xFF��
** Version:	
*******************************************************************************/
void ReplyTrue(void)
{
  	unsigned char ReplyUsart[21] = {0x68};
	unsigned char ReplyTemp[8] = {0xD2,0x00,0x05,0x00,0x00,0x00,0x00,0xFF};
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//����������
	memcpy(&ReplyUsart[11], ReplyTemp, 8);	//������
	  
	a[0] = check_bit(ReplyUsart, 19);
	memcpy(&ReplyUsart[19], a, 2);
	Ext_Wdog_Feed();

  	Send_232_data(ReplyUsart, 21);
		
 	Ext_Wdog_Feed();
}
/*******************************************************************************
** Name:		void ReplyFalse(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	��ִ����ɽ��մ���󣬻ظ�0x00��
** Version:	
*******************************************************************************/
void ReplyFalse(void)
{
	unsigned char ReplyUsart[21] = {0x68};
	unsigned char ReplyTemp[8] = {0xD2,0x00,0x05,0x00,0x00,0x00,0x00,0x00};
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//����������
	memcpy(&ReplyUsart[11], ReplyTemp, 8);		
	  
	a[0] = check_bit(ReplyUsart, 19);
	memcpy(&ReplyUsart[19], a, 2);
	Ext_Wdog_Feed();

  	Send_232_data(ReplyUsart, 21);
		
 	Ext_Wdog_Feed();
}
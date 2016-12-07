
#include "includes.h"

//*************************以下定义与主机发送的命令相关**************************
#define FRAMEBUF_SIZE 22					//最大帧长度
#define IDLELINE_TIME 10					//线路空闲判断时间
unsigned char FrameBuff[FRAMEBUF_SIZE];	//接收帧缓冲区
unsigned char UART_RcvCnt = 0;				//接收计数
//******************************************************************************

//static unsigned char RecevieBuf[24] = {0};	//900HP接收到的指令存储到该数组。左对齐。
extern volatile unsigned int FlagUSCIA1;
extern unsigned char SamplingPeriod[2], SamplingPoint[2], SamplingFrequency[2];;
extern uchar TerminalNum[6], SensorNum[4];
extern uchar a[2];
extern float Temperature , BatteryVoltage;
extern float maxz, maxiz;
extern float ReplyZgDatas[512];	//存储波形信号;
typedef union FLOAT
{
  	float f_data;
  	uchar hex_float[4];
}uf;
uf hex_float4;

/******************************************************************************
** 函数名称：void Usart_GPIO_Init()  
** 功    能：串口IO口初始化
** 修改日志：
*******************************************************************************/
void Usart_GPIO_Init(void) 
{
  	USART_TX_232_SEL;
  	USART_RX_232_SEL;
}
/******************************************************************************
** 函数名称：void Usart_Wire_Temp_Init_9600(void)
** 功    能：串口A0初始化 ,按照不同要求的波特率进行初始化
** 修改日志：
*******************************************************************************/
void Usart_Wire_Temp_Init_9600(void)
{
//    UCA1CTL1 &=~UCSWRST;              // **Put state machine in reset**
//    UCA1CTL1 &=~UCSSEL__SMCLK;        // SMCLK

    UCA1CTL1 |= UCSWRST;              // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL__SMCLK;        // SMCLK

    UCA1BR0 = 0X23;                   //baud 设置参考 user's guide Page 954
    UCA1BR1 = 0x08;					// 20MHz/(8*256+37) = 9600

    UCA1MCTL =0X00;                  // Modulation
    UCA1CTL1 &= ~UCSWRST;            // **Initialize USCI state machine**
    return;
}
/*****************************************************************************
** 函数名称：Send_232_data(uchar *tdata,uint data_length);
** 功    能：232数据发送 UART_A1
** 修改日志：
*****************************************************************************/
void Send_232_data(unsigned char *tdata, unsigned int data_length)
{
//   while (!(UCA1IFG & UCTXIFG));
   for(unsigned char s = 0; s < data_length; s++)
   {
    	while(!(UCA1IFG & UCTXIFG));	// 判断是否发送完毕
    	Ext_Wdog_Feed();
		UCA1TXBUF = *(tdata + s);
   }
   while (!(UCA1IFG & UCTXIFG));		// 判断是否发送完毕  
}
/*************************************************************************
** Name:		void Receive_232_data(void)
** Workspace: 	IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 		2015.07.27
** Function:	接收主机指令，根据不同指令返回不同信息，
**				详见"G:\HUIYUAN COMMUNICATION\01微风振动\文档\微风振动传感器通讯规约.doc"		
** Version:	
*************************************************************************/
void Receive_232_data(void)
{
	UCA1CTL1 &= ~UCSWRST;                                        // 使能串口功能
    UCA1IE |= UCRXIE;                                            // 使能接收中断
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
				FrameBuff[UART_RcvCnt] = UCA1RXBUF;	//接收到的字节存入缓冲
				UART_RcvCnt++;						//计数+1
			}
			else
			{
				UCA1IE &= ~UCRXIE;				//达到缓冲区大小后，禁止串口接收中断
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
** Function:	控制字	命令
				C0H		配置传感器号码
				C1H		设置采样参数
				C2H		请求振动幅值和频率
				C3H		请求振动波形信号
				C4H		请求电池电压和温度
	判断过程	：	首先判断接收到的数据校验码是否正确；
				再判断接收到的数据中传感器号码是否正确；
				返回OK；
				开始根据命令修改系统设置。
** Version:	
*************************************************************************/
void RecevieBufReading(unsigned char *RxBuf)
{
  	unsigned char i = 0, FlagRxCheck = 0, FlagSensorNumCheck = 0;
	unsigned char RxSensorNum[4] = {0};
//*************************校验接收数据是否正确**********************************
	FlagRxCheck = RxCheck(RxBuf);
	for(i = 0; i < 4; i++)
	{
		RxSensorNum[i] = RxBuf[i + 7];		//存储接收数据中的传感器号码
	}
	if(FlagRxCheck == 0)
	{
		FlagSensorNumCheck = memcmp(RxSensorNum, SensorNum, 4);	//校验传感器号码是否一样，相等返回0
		if(FlagSensorNumCheck == 0)
		{
		  	ReplyTrue();
			switch(RxBuf[11])	//根据控制字判断指令
			{
		  		case 0xC0:
				  	for(i = 0; i < 4; i++)
					{
						SensorNum[i] = RxBuf[i + 14];
					}
//******************************修改传感器号码，保存到EEPROM*********************					
					P2OUT &= ~BIT5;
					EepromWritePage(0x01, 	SensorNum, 	4);	
					P2OUT |= BIT5;
				  	break;
				case 0xC1:	
//						SamplingPeriod = RxBuf[14] * 256 + RxBuf[15];		//得到采样周期，单位分钟
//						SamplingPoint = RxBuf[16] * 256 + RxBuf[17];		//得到采样点数，单位点
//						SamplingFrequency = RxBuf[18] * 256 + RxBuf[19];	//得到采样频率,单位Hz						
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
//******************************修改采样参数，保存到EEPROM***********************					
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
** Function:	回复振动幅值和振动频率。
** Version:	
*************************************************************************/
void ReplyAmplitudeAndFrequency(void)
{
	unsigned char ReplyUsart[26] = {0x68};
	unsigned char Temp[4] = {0};
	unsigned char ReplyTemp[7] = {0xD0,0x00,0x0a,0x00,0x00,0x00,0x00};
	unsigned char i = 0;
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//传感器号码
	memcpy(&ReplyUsart[11], ReplyTemp, 7);		//
	
	hex_float4.f_data = maxiz;		//得到振动幅值
	for(i = 2; i < 4; i++)			//hex_float4.hex_float（4字节）。VibrationAmplitude（2字节）
	{
			Temp[i - 2] = hex_float4.hex_float[5 - i];
	}
	memcpy(&ReplyUsart[18], Temp, 2);
	
	hex_float4.f_data = maxz;		//得到振动频率
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
** Function:	回复振动波形。即加速度的原始采样值。数据域保留64字节
** Version:	
*************************************************************************/
void ReplyWaveform(void)
{
  	unsigned char ReplyUsart[80] = {0x68};
	unsigned char ReplyTemp[3] = {0xD1,0x00,0x40};
	unsigned i = 0; 
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//传感器号码
	memcpy(&ReplyUsart[11], ReplyTemp, 3);		//
//*******返回的波形数据格式如“D0H”，实际计算时应作为“D0 00 00 00”计算*********	
	for(i = 0; i < 64; i++)						
	{
		hex_float4.f_data = ReplyZgDatas[i];
		memcpy(&ReplyUsart[14 + i], &hex_float4.hex_float[3], 1);	//波形数据保留64字节
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
** Function:	回复电池电压和温度数据。
** Version:	
*************************************************************************/
void ReplyVoltageAndTemperature(void)
{
	unsigned char ReplyUsart[24] = {0x68};
	unsigned char ReplyTemp[3] = {0xD2,0x00,0x08};//0x40 —> 64个波形数据
	unsigned char Temp[4] = {0}, i = 0;
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//传感器号码
	memcpy(&ReplyUsart[11], ReplyTemp, 3);	//
	
	hex_float4.f_data = BatteryVoltage;	//得到电池电压
	for(i = 0; i < 4; i++)		
	{
		Temp[3 - i] = hex_float4.hex_float[i];
	}
	memcpy(&ReplyUsart[14], Temp, 4);
	hex_float4.f_data = Temperature;		//得到温度
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
** Function:	在执行完成接收指令后，回复0xFF。
** Version:	
*******************************************************************************/
void ReplyTrue(void)
{
  	unsigned char ReplyUsart[21] = {0x68};
	unsigned char ReplyTemp[8] = {0xD2,0x00,0x05,0x00,0x00,0x00,0x00,0xFF};
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//传感器号码
	memcpy(&ReplyUsart[11], ReplyTemp, 8);	//控制字
	  
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
** Function:	在执行完成接收错误后，回复0x00。
** Version:	
*******************************************************************************/
void ReplyFalse(void)
{
	unsigned char ReplyUsart[21] = {0x68};
	unsigned char ReplyTemp[8] = {0xD2,0x00,0x05,0x00,0x00,0x00,0x00,0x00};
	
	memcpy(&ReplyUsart[1], TerminalNum, 6);
	memcpy(&ReplyUsart[7], SensorNum, 4);		//传感器号码
	memcpy(&ReplyUsart[11], ReplyTemp, 8);		
	  
	a[0] = check_bit(ReplyUsart, 19);
	memcpy(&ReplyUsart[19], a, 2);
	Ext_Wdog_Feed();

  	Send_232_data(ReplyUsart, 21);
		
 	Ext_Wdog_Feed();
}
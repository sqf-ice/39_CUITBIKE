/*******************************************************************************
** 版权:     	xxxlzjxxx CO.,LTD
** 文件名:   	main.c
** 工作环境: 	IAR 6.10
** 工程大小:	
** 作者:     	xxxlzjxxx
** 生成日期:	    2016年12月01日 
** 功能:       CUITBIKE_V12_MSP430-SV10_161201_D
				成信车队码表（msp430版）测试程序
								  msp430f5529
-------------------------------------------------------------------------------
TPS3823:     WDI ->P1.0

BATTERY:    ADC_BAT->P6.0
BAT_ISET2:  BAT_ISET2->P5.7

NRF24L01:   CE  ->P2.6, CSN->P2.7， SCK->P3.2，  MOSI->P3.0，  MISO->P3.1， IRQ->P2.5
NEO-6M:     RXD ->P4.4， TXD->P4.5, PPS->P5.6
BLUTOOTH:   RXD ->P3.3,  TXD->P3.4，  EN->P3.5,  BT_LED->P3.6
W25Qxx:     CS  ->P4.0,CLK->P4.3,  MISO->P4.2, MOSI->P4.1
SENSOR_EN:  SENSOR_EN->P3.7
BMP180:     SCL ->P2.4, SDA->P2.3
AT24C512:   SDA ->P1.6, SCL->P1.7
PCF85163:   INT ->P2.2, SDA->P2.0,  SCL->P2.1
JLX12864:   EN->P6.7, CS->P6.6, RESET->P6.5, RS->P6.4, SDA->P6.3, SCL->P6.2,    
			ROM_CS->P7.3, ROM_SCK->P7.2, ROM_OUT->P7.1, ROM_IN->P7.0
LED:        LED2->P4.6
            LED3->P4.7
SWITCH:		 SW1->RESET
			 SW2->3455_ON
            SW3->P1.0(BACK)
            SW4->P1.1(ENTER)
            SW5->P1.2(MENU)
            SW6->P3.7(UP)
            SW7->P3.6(DOWN)
BEEP:       BEEP->P7.4
-------------------------------------------------------------------------------
** 相关器件:
            MCU
** 修改日志：
			
*******************************************************************************/
/*********************************包含头文件***********************************/

#include "includes.h"

/*********************************子函数调用***********************************/
void ReadConfigInformation(void);
void Process(void);
float GetBatteryVoltage(void);
float GetTemperature(void);
/***********************************定义变量***********************************/
float BatteryVoltage = 0, Temperature = 0;
unsigned int Flag_Timer1A0 = 0;
char adcvalue[5] = {1,2,3,4,5};
unsigned char string = 0x01;

extern const unsigned char  bmp1[], page1[], page2[], page3[];
/*******************************主函数*****************************************/
void main( void )
{  
    WDOG_Init(0); 
    SystemClock(); 
    GPIO_Init();
    
//    USBCNF &= ~USB_EN;
    
    Init_TimerB0();
    ADC12_Init();
    
    _EINT();
//读取EEPROM的相关配置信息	
//	Ext_Wdog_Feed();
//	ReadConfigInformation();
//    Ext_Wdog_Feed();
//	
//	Usart_GPIO_Init();
//    Usart_Wire_Temp_Init_9600();
//	Send_232_data(y, 1);
//	Send_232_data(x, 2);
//	Send_232_data(y, 1);
	Initial_lcd();
	display_128x64(bmp1);
    delay_ms(500);
    clear_screen();
    
	while(1)
	{
		 display_128x64(page1);
         delay_ms(2000);
         display_128x64(page2);
         delay_ms(2000);
         display_128x64(page3);
         delay_ms(2000);
	}
}
/*******************************************************************************
** Name:			void ReadConfigInformation(void)
** Workspace: 		IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 			2015.10.13
** Function:		读取eeprom中的配置信息
** Input:			
** Output:			
** Version:	
*******************************************************************************/
void ReadConfigInformation(void)
{
//  	P2OUT &= ~BIT5;		//Unable WriteProtect 	
//	EepromSequentialRead(0x00,	TerminalNum, 		6);	//default: {0x48,0x59,0x30,0x30,0x30,0x31}
//	EepromSequentialRead(0x01, 	SensorNum, 			4);
//	EepromSequentialRead(0x02, 	SamplingPeriod, 	2);	//default:	{0x00,0x00,0x00,0x0a} --> 10分钟
//	EepromSequentialRead(0x03, 	SamplingPoint, 		2);	//default:	{0x00,0x00,0x02,0x00} --> 512点
//	EepromSequentialRead(0x04, 	SamplingFrequency, 	2);	//default:	{0x00,0x00,0x02,0x00} --> 512Hz	
//	P2OUT |= BIT5;		//Enable WriteProtect
//	
//	Ext_Wdog_Feed();
////**************************将两字节16进制转化为10进制***************************	
//	FT = (SamplingPeriod[0] 	<< 8) | SamplingPeriod[1];
//	FP = (SamplingPoint[0] 		<< 8) | SamplingPoint[1];
//	FS = (SamplingFrequency[0] 	<< 8) | SamplingFrequency[1];	
}
/*******************************************************************************
** Name:			void LTC3455_Init(void)
** Workspace: 		IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 			2015.12.9
** Function:		配置电源管理芯片LTC3455的工作方式
                  LTC3455:    MODE->P7.5, HSON->P7.6, ON2->P7.7，PWRON->P8.0，PBSTAT->P2.4
** Input:			
** Output:			
** Version:	
*******************************************************************************/
void LTC3455_Init(void)
{
    P7OUT |= BIT5;					//burst模式：高效率高纹波
    P7OUT &=~ BIT6;					//热插拔关闭使能
    P8OUT |= BIT0;					//电源使能
	P7OUT |= BIT7;					//VCC2使能，PWRON拉高后ON2拉高才有效
}
/*******************************************************************************
** Name:			__interrupt void TIMER1_A0_ISR(void)
** Workspace: 		IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 			2015.07.31
** Function:		
** Input:			
** Output:			
** Version:	
*******************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{   
    LPM3_EXIT;
//    Ext_Wdog_Feed();
    Flag_Timer1A0++;
    if(Flag_Timer1A0  == 5)
    {
        Flag_Timer1A0 = 0;
    }
}
/*******************************************************************************
** Name:			void Process(void)
** Workspace: 		IAR 6.10 
** Designed by:	xxxlzjxxx
** Date: 			2015.07.31
** Function:		完成系统任务
** Input:			
** Output:			
** Version:		
*******************************************************************************/
void Process(void)
{
	BatteryVoltage = GetBatteryVoltage();
    Temperature    = GetTemperature();
    
    P7OUT ^= BIT2 + BIT3;
    P8OUT ^= BIT0 + BIT1;
}
/*******************************************************************************
** Name:			float GetBatteryVoltage(void)
** Return:        返回采集的电压值
** Function:		完成系统任务
                    100%----4.20V 
                　　90%-----4.06V 
                　　80%-----3.98V 
                　　70%-----3.92V 
                　　60%-----3.87V 
                　　50%-----3.82V 
                　　40%-----3.79V 
                　　30%-----3.77V 
                　　20%-----3.74V 
                　　10%-----3.68V 
                　　5%------3.45V 
                　　0%------3.00V
** Input:			
** Output:			
** Version:		
*******************************************************************************/
float GetBatteryVoltage(void)
{ 	
  	unsigned char i;
	float TempAvg = 0, Temp = 0;

  	for(i = 0; i < 5; i++)
	{
  		ADC12CTL0|=ADC12SC;
  		while(!(ADC12IFG & BIT0));
  		Temp = (float)ADC12MEM0 / 4096.0 * 3.17 * 2;  //*2为电阻分压比例，f_data为实际电压值
		TempAvg = TempAvg + Temp;
		
//		Ext_Wdog_Feed();
	}
	TempAvg = TempAvg / 5;

	return TempAvg;
} 
/******************************************************************************
** 函数名称： float GetTemperature()
** 功    能:  采集AD7414-0的温度，采样5次(删除最大值和最小值)再求均值
** 修改日志：
******************************************************************************/
float GetTemperature(void)
{
//  	float TmpMax = -55, TmpMin = 125, TmpAvg = 0, Tmpre = 0;
//	unsigned char i;
//	
//  	for(i = 0; i < 5; i++)
//	{
//  		Tmpre = AD7414_Read_Temp();
//		TmpAvg = TmpAvg + Tmpre;
//		if(Tmpre >= TmpMax)
//		{
//			TmpMax = Tmpre;
//		}
//		if(TmpMin >= Tmpre)
//		{
//			TmpMin = Tmpre;
//		}
//		
//		Ext_Wdog_Feed();
//	}
//	TmpAvg = (TmpAvg - TmpMax - TmpMin) / 3;
//  	Ext_Wdog_Feed();
//	
//	return TmpAvg
    return 0;
}


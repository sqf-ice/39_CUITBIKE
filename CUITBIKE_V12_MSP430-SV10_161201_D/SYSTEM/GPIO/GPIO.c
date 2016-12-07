/*******************************************************************************
** 版权:     	xxxlzjxxx CO.,LTD
** 文件名:   	GPIO.c
** 工作环境: 	IAR 6.10
** 工程大小:	
** 作者:     	xxxlzjxxx
** 生成日期:	    201６年12月01日 
** 功能:       IO初始化
** 相关器件:
** 修改日志：
			
*******************************************************************************/
/*********************************包含头文件************************************/

#include "includes.h"

/******************************************************************************/
/*******************************************************************************
** Name:			void GPIO_Init(void)
** Workspace: 		IAR 6.10 
** Designed by:	    xxxlzjxxx
** Date: 			2016.12.01
** Function:		配置IO口，未使用的全部输出为低电平。默认关闭全部模块电源
** Version:	
*******************************************************************************/
void GPIO_Init()
{
//设置I/O口为输出     
	P1DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7; 
	P2DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P3DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P4DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P5DIR |= BIT0|BIT1|BIT6|BIT7;
	P6DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P7DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P8DIR |= BIT0|BIT1|BIT2;
//设置I/O口输出低电平  
	P1OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); 
	P2OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P3OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P4OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P5OUT &= ~(BIT0|BIT1|BIT6|BIT7);
	P6OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P7OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P8OUT &= ~(BIT0|BIT1|BIT2);
//设置ACLK输出，该IO口是看门狗喂狗的IO   
    P1DIR |= BIT0;                            // ACLK set out to pins
    P1SEL |= BIT0;
//***************************以下设置需要使用的IO口****************************
    LCD_ON;		    //开启LCD12864电源
	LCD_LED_ON;		//开启LCD12864背光
//开启2个LED
    LED0_ON;
    LED1_ON;
}

//void Ext_Wdog_Feed(void)
//{
//    P5OUT ^= BIT7;     //System WDOG
//	P5OUT ^= BIT7;     //System WDOG
//}


/*******************************************************************************
** ��Ȩ:     	xxxlzjxxx CO.,LTD
** �ļ���:   	GPIO.c
** ��������: 	IAR 6.10
** ���̴�С:	
** ����:     	xxxlzjxxx
** ��������:	    201����12��01�� 
** ����:       IO��ʼ��
** �������:
** �޸���־��
			
*******************************************************************************/
/*********************************����ͷ�ļ�************************************/

#include "includes.h"

/******************************************************************************/
/*******************************************************************************
** Name:			void GPIO_Init(void)
** Workspace: 		IAR 6.10 
** Designed by:	    xxxlzjxxx
** Date: 			2016.12.01
** Function:		����IO�ڣ�δʹ�õ�ȫ�����Ϊ�͵�ƽ��Ĭ�Ϲر�ȫ��ģ���Դ
** Version:	
*******************************************************************************/
void GPIO_Init()
{
//����I/O��Ϊ���     
	P1DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7; 
	P2DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P3DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P4DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P5DIR |= BIT0|BIT1|BIT6|BIT7;
	P6DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P7DIR |= BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7;
	P8DIR |= BIT0|BIT1|BIT2;
//����I/O������͵�ƽ  
	P1OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); 
	P2OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P3OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P4OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P5OUT &= ~(BIT0|BIT1|BIT6|BIT7);
	P6OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P7OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
	P8OUT &= ~(BIT0|BIT1|BIT2);
//����ACLK�������IO���ǿ��Ź�ι����IO   
    P1DIR |= BIT0;                            // ACLK set out to pins
    P1SEL |= BIT0;
//***************************����������Ҫʹ�õ�IO��****************************
    LCD_ON;		    //����LCD12864��Դ
	LCD_LED_ON;		//����LCD12864����
//����2��LED
    LED0_ON;
    LED1_ON;
}

//void Ext_Wdog_Feed(void)
//{
//    P5OUT ^= BIT7;     //System WDOG
//	P5OUT ^= BIT7;     //System WDOG
//}


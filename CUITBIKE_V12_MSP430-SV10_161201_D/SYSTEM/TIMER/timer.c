/*****************************************************************************
** �ļ����ƣ�timer.c
** ��    �ܣ�
** �޸���־��
******************************************************************************/

#include "includes.h"

/******************************************************************************
** �������ƣ� void Init_TimerA(void)
** ��    ��:  ��ʱ��A�ĳ�ʼ��,ÿ��6s����LPMxģʽ�µ�MCU
** �޸���־��
******************************************************************************/
void Init_TimerA(void)
{
   	TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
   	TA1CCR0 = 512 - 1;			            // 1sһ���ж�
   	TA1EX0 = TAIDEX_7;                      //timer A ����8��Ƶ,����ACLKΪ32768/8/8=512HZ ,(31.25ms)
   	TA1CTL = TASSEL_1 + MC_1 + TACLR + ID_3; //with ACLK(32768) as clock ,up mode,����8��Ƶ divide
}
/******************************************************************************
** �������ƣ� void Init_TimerB(void)
** ��    ��:  ��ʱ��B�ĳ�ʼ��
** �޸���־��  
******************************************************************************/
void Init_TimerB0(void)
{
    TB0CCTL0 = CCIE;                           // TRCCR0 interrupt enabled
    TB0CCR0 = 32768 - 1;                         //10sһ���ж�
    TB0CTL = TBSSEL_1 + MC_1 + TBCLR;          // ACLK, upmode, clear TBR
}
// Timer B0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMERB0_VECTOR))) TIMERB1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P4OUT ^= BIT7;                          // Toggle P1.0 using exclusive-OR
}


/*****************************************************************************
** 文件名称：timer.c
** 功    能：
** 修改日志：
******************************************************************************/

#include "includes.h"

/******************************************************************************
** 函数名称： void Init_TimerA(void)
** 功    能:  定时器A的初始化,每隔6s唤醒LPMx模式下的MCU
** 修改日志：
******************************************************************************/
void Init_TimerA(void)
{
   	TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
   	TA1CCR0 = 512 - 1;			            // 1s一次中断
   	TA1EX0 = TAIDEX_7;                      //timer A 级联8分频,最终ACLK为32768/8/8=512HZ ,(31.25ms)
   	TA1CTL = TASSEL_1 + MC_1 + TACLR + ID_3; //with ACLK(32768) as clock ,up mode,输入8分频 divide
}
/******************************************************************************
** 函数名称： void Init_TimerB(void)
** 功    能:  定时器B的初始化
** 修改日志：  
******************************************************************************/
void Init_TimerB0(void)
{
    TB0CCTL0 = CCIE;                           // TRCCR0 interrupt enabled
    TB0CCR0 = 32768 - 1;                         //10s一个中断
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


/*******************************************************************************
** 版权:     	xxxlzjxxx CO.,LTD
** 文件名:   	sysclk.c
** 工作环境: 	IAR 6.10
** 工程大小:	
** 作者:     	xxxlzjxxx
** 生成日期:	    2016年12月01日 
** 功能:       系统时钟设置
** 相关器件:
** 修改日志：
			
*******************************************************************************/
/*********************************包含头文件************************************/

#include "includes.h"

/******************************************************************************/
/*******************************************************************************
** Name:			void SystemClock(void)
** Workspace: 		IAR 6.10 
** Designed by:	    xxxlzjxxx
** Date: 			2015.12.9
** Function:		系统时钟初始化
                    MCLK = SMCLK = XT2 = 25MHz
                    ACLK = XT = 32768Hz
** Version:	
*******************************************************************************/
void  SystemClock(void)
{
    P5SEL |= BIT2 + BIT3 + BIT4 + BIT5;      // Port select XT1 XT2

    UCSCTL6 &= ~(XT1OFF + XT2OFF);                       // Enable XT2 
    UCSCTL6 |= XCAP_3;                        // Internal load cap
    UCSCTL3 |= SELREF_0;                      // FLLref = XT1
                                            // Since LFXT1 is not used,
                                            // sourcing FLL with LFXT1 can cause
                                            // XT1OFFG flag to set
    UCSCTL4 |= SELA_0;                        // ACLK=XT1,SMCLK=DCO,MCLK=DCO

    // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                            // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag

    UCSCTL6 &= ~(XT1DRIVE_3 + XT2DRIVE_3);   //Xtal is now stable, reduce drive strength
                                            // Decrease XT2 Drive according to
                                            // expected frequency
    UCSCTL4 |= (SELA_0 + SELS_5 + SELM_5);   // SMCLK=MCLK=XT2
}


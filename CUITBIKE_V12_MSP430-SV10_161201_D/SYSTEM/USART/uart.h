/** �޸���־��
******************************************************************************/
#ifndef __UART_H
#define __UART_H

/*****************************************************************************
** �ܽŶ���
*****************************************************************************/
#define USART_TX_232_SEL P5SEL|=BIT6 //232���Ͷ˿�
#define USART_RX_232_SEL P5SEL|=BIT7 //232���ն˿�                 
/*******************************************************************************
** ��������
*******************************************************************************/
void Send_232_data(uchar *tdata,uint data_length);
void Usart_Wire_Temp_Init_9600(void);
void Usart_GPIO_Init(void) ;
void Send_Wave_Check(void);

void Receive_232_data(void);
void RecevieBufReading(unsigned char *RxBuf);
void ReplyTrue(void);
void ReplyFalse(void);
void ReplyAmplitudeAndFrequency(void);
void ReplyWaveform(void);
void ReplyVoltageAndTemperature(void);


#endif
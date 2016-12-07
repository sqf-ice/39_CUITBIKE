/*
 * LCD12864.h
 *
 *  Created on: 2014-11-29
 *      Author: dell
 */

#ifndef __LCD12864_H
#define __LCD12864_H

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long

#define CS_PIN           (1<<6) //P7.4 CS
#define RST_PIN          (1<<5) //P7.3 RESET
#define RS_PIN           (1<<4) //P7.2 RS
#define SPI_CLK_PIN      (1<<2) //P7.0 SCLK
#define SPI_MOSI_PIN     (1<<3) //P7.1 DIN/SDA

#define Rom_OUT_BIT P7IN&BIT2

#define Rom_IN      	(1<<1) //P5.0 Rom_IN
#define Rom_OUT     	(1<<2) //P5.1 Rom_OUT
#define Rom_SCK         (1<<3) //P5.2 Rom_SCK
#define Rom_CS          (1<<0) //P5.3 Rom_CS

#define DIN_0()                   P6OUT &= ~SPI_MOSI_PIN
#define DIN_1()                   P6OUT |= SPI_MOSI_PIN
#define SCLK_0()                  P6OUT &= ~SPI_CLK_PIN
#define SCLK_1()                  P6OUT |= SPI_CLK_PIN
#define CS_0()				      P6OUT &= ~CS_PIN //CS_PIN=0
#define CS_1()				      P6OUT |= CS_PIN //CS_PIN=0
#define RS_0()                    P6OUT &= ~RS_PIN//RS=0
#define RS_1()                    P6OUT |= RS_PIN//RS=1
#define RST_0()                   P6OUT &= ~RST_PIN
#define RST_1()                   P6OUT |= RST_PIN

#define Rom_IN_0()                  P7OUT &= ~Rom_IN
#define Rom_IN_1()                  P7OUT |= Rom_IN
#define Rom_OUT_0()				    P7OUT &= ~Rom_OUT //CS_PIN=0
#define Rom_OUT_1()				    P7OUT |= Rom_OUT //CS_PIN=0
#define Rom_SCK_0()                 P7OUT &= ~Rom_SCK//RS=0
#define Rom_SCK_1()                 P7OUT |= Rom_SCK//RS=1
#define Rom_CS_0()                  P7OUT &= ~Rom_CS
#define Rom_CS_1()                  P7OUT |= Rom_CS


void delayms(int n_ms);
void delayus(int n_us);

void transfer_command_lcd(int data1);
void transfer_data_lcd(int data1);
void Initial_lcd(void);
void lcd_address(uint page,uint column);
void clear_screen();
void display_128x64(const uchar *dp);
void display_graphic_16x16(uchar page,uchar column,const uchar *dp);
void display_graphic_8x16(uchar page,uchar column,const uchar *dp);
void display_graphic_5x8(uchar page,uchar column,const uchar *dp);
void send_command_to_ROM( uchar datu );
static uchar get_data_from_ROM( );
void get_and_write_16x16(ulong fontaddr,uchar page,uchar column);
void get_and_write_8x16(ulong fontaddr,uchar page,uchar column);
void get_and_write_5x8(ulong fontaddr,uchar page,uchar column);
void display_GB2312_string(uchar page,uchar column,char *text);
void display_string_5x8(uchar page,uchar column,char *text);

#endif /* __LCD12864_H */


#ifndef __GPIO_H
#define __GPIO_H

/********************************************/
#define SENSOR_ON               (P3OUT |= BIT7)
#define SENSOR_OFF              (P3OUT &= ~BIT7)

#define BAT_ISET_500MA          (P5OUT |= BIT7)
#define BAT_ISET_800MA          (P5OUT &= ~BIT7)

#define BLE_ON                  (P3OUT |= BIT5)
#define BLE_OFF                 (P3OUT &= ~BIT5)

#define LCD_LED_ON              (P6OUT |= BIT1)
#define LCD_LED_OFF             (P6OUT &= ~BIT1)

#define LCD_ON                  (P6OUT |= BIT7)
#define LCD_OFF                 (P6OUT &= ~BIT7)

#define LED0_OFF                (P4OUT |= BIT6)
#define LED0_ON                 (P4OUT &= ~BIT6)
#define LED0_OVERTURN           (P4OUT ^= BIT6)

#define LED1_OFF                (P4OUT |= BIT7)
#define LED1_ON                 (P4OUT &= ~BIT7)
#define LED1_OVERTURN           (P4OUT ^= BIT7)
/********************************************/
void GPIO_Init();
//void Ext_Wdog_Feed(void);


#endif


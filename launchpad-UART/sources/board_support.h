#ifndef __BOARD_SUPPORT_H__
#define __BOARD_SUPPORT_H__

// This is the SMCLK frequency. Pay attention because it drift with Temperature and Vcc changes.
//#define SMCLK_FREQUENCY           980000uL
#define SMCLK_FREQUENCY           1000000uL

//Pinout definition.
// PORT 1
#define ACT_LED_PIN     BIT0                // Activity led pin on P1.0
#define TXD             BIT1                // TXD on P1.1
#define RXD             BIT2                // RXD on P1.2
#define PUSH_BUTTON_PIN BIT3                // Push button pin on P1.3

//Faster access to pin
#define ACT_LED     P1OUT_bit.P1OUT_0       



#endif //__BOARD_SUPPORT_H__
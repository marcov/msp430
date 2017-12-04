#ifndef __BOARD_SUPPORT_H__
#define __BOARD_SUPPORT_H__

// This is the SMCLK frequency. Pay attention because it drift with Temperature and Vcc changes.
//#define SMCLK_FREQUENCY           980000uL
#define SMCLK_FREQUENCY           1000000uL
#define MCLK_FREQUENCY            SMCLK_FREQUENCY

//Pinout definition.
// PORT 1
#define ACT_LED_PIN     BIT0                // Activity led pin on P1.0
#define TXD             BIT1                // TXD on P1.1
#define RXD             BIT2                // RXD on P1.2
#define PUSH_BUTTON_PIN BIT3                // Push button pin on P1.3


//Faster access to pin
#define ACT_LED     P1OUT_bit.P1OUT_0       


/*-------1-wire definitions-------*/
#define ONE_WIRE_PORT_PIN	5
// Dont change the following!
#define ONE_WIRE_PIN_WRITE	P1OUT_bit.P1OUT_5
#define ONE_WIRE_PIN_READ       P1IN_bit.P1IN_5
#define ONE_WIRE_PIN_DIR	P1DIR_bit.P1DIR_5

#endif //__BOARD_SUPPORT_H__
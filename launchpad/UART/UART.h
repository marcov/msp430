#ifndef __UART_H__
#define __UART_H__

// Set the baudrate here:
#define BAUDRATE        9600

//Set this to 1 to have LEDs visual feedback for TX and RX activity.
#define USE_LEDS_TXRX   1


/*----------------------------------------------------------------------------*/
// Don't edit below this.

#if USE_LEDS_TXRX

#define LED_TX_PIN  0
#define LED_RX_PIN  6
#define LEDS_PORT   1

// All this wrapper marcro-functions are needed in order to make Expansion to
// #defined values above work!

#define LED_TX            LED_OUT(LEDS_PORT, LED_TX_PIN)
#define LED_TX_DIR        LED_DIR(LEDS_PORT, LED_TX_PIN)
#define LED_RX            LED_OUT(LEDS_PORT, LED_RX_PIN)
#define LED_RX_DIR        LED_DIR(LEDS_PORT, LED_RX_PIN)

#define SETUP_UART_LEDS()  \
  LED_TX = 0; \
  LED_RX = 0; \
  LED_TX_DIR = 1; \
  LED_RX_DIR = 1

#define LED_OUT(PORT,PIN)     OUT_BIT(PORT,PIN)
#define LED_DIR(PORT,PIN)     DIR_BIT(PORT,PIN)

#define DIR_BIT(PORT,PIN)  \
        (P ## PORT ## DIR_bit.P ## PORT ## DIR_ ## PIN)

#define OUT_BIT(PORT,PIN)  \
        (P ## PORT ## OUT_bit.P ## PORT ## OUT_ ## PIN)

#else

#define SETUP_UART_LEDS()
#define LED_TX
#define LED_RX

#endif


void initUART(void);
void UART_echo_mode(void);
void UART_tx_string (char * str);
void start_UART_rx(void);
unsigned char UART_get_char(void);

#endif //__UART_H__
#ifndef __UART_H__
#define __UART_H__

#define BAUDRATE        9600
#define USE_LEDS_TXRX   1

#if USE_LEDS_TXRX


#define LED_TX_OUT  0
#define LED_RX_PIN  6
#define LEDS_PORT   1


/*
#define LED_TX_PIN  0
#define LED_RX_PIN  6
#define LEDS_PORT   1
*/
//Is it possible to substitute numbers below with the definitions above???

#define LED_TX            OUT_BIT(1, 0)
#define LED_TX_DIR        DIR_BIT(1, 0)
#define LED_RX            OUT_BIT(1, 6)
#define LED_RX_DIR        DIR_BIT(1, 6)
  
#define SETUP_UART_LEDS()  \
  LED_TX = 0; \
  LED_RX = 0; \
  LED_TX_DIR = 1; \
  LED_RX_DIR = 1

#define DIR_BIT(PORT,PIN)  \
        (P ## PORT ## DIR_bit.P ## PORT ## DIR_ ## PIN)

#define OUT_BIT(PORT,PIN)  \
        (P ## PORT ## OUT_bit.P ## PORT ## OUT_ ## PIN)

#else

#define SETUP_UART_LEDS()

#endif


void initUART(void);
void UART_echo_mode(void);
void UART_tx_string (char * str);
void start_UART_rx(void);
unsigned char UART_get_char(void);

#endif //__UART_H__
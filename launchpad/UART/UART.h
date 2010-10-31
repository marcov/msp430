#ifndef __UART_H__
#define __UART_H__

// Set the baudrate here:
#define BAUDRATE        4800

//Set this to 1 to have LEDs visual feedback for TX and RX activity.
#define USE_LEDS_TXRX   1

//Enter the Leds pin number 0-7 and the leds port.
#define LED_TX_PIN  0  
#define LED_RX_PIN  6
#define LEDS_PORT   1

//Size of input buffer
#define UART_RX_BUFFER_SIZE    16
//Set to 0 to use the stack-style input buffer, instead.
#define USE_RX_RINGBUFFER       1


/*----------------------------------------------------------------------------*/
// Don't edit below this.
#if USE_LEDS_TXRX

// Use this macros to access led pins faster.
#define LED_TX            LED_OUT(LEDS_PORT, LED_TX_PIN)
#define LED_TX_DIR        LED_DIR(LEDS_PORT, LED_TX_PIN)
#define LED_RX            LED_OUT(LEDS_PORT, LED_RX_PIN)
#define LED_RX_DIR        LED_DIR(LEDS_PORT, LED_RX_PIN)

#define SETUP_UART_LEDS()  \
  LED_TX = 0; \
  LED_RX = 0; \
  LED_TX_DIR = 1; \
  LED_RX_DIR = 1

// All these macro wrapper functions are needed in order to make expansions to
// #defined values above work!

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

/*----------------------------------------------------------------------------*/
//Functions prototypes.

void initUART(void);
void UART_echo_mode(void);
void UART_puts (unsigned char * str);
unsigned char UART_getch(void);
void UART_putch(unsigned char chr);
void UART_putbuffer (unsigned char * txbuffer, unsigned char length);

#endif //__UART_H__
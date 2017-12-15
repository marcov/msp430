#ifndef __UART_H__
#define __UART_H__

// Set the baudrate here:
#define BAUDRATE        4800

//Set this to 1 to have LEDs visual feedback for TX and RX activity.
#define USE_LEDS_TXRX   1

//Enter the Leds pin number 0-7 and the leds port.
#define LED_TX  0
#define LED_RX  6
#define LEDS_PORT   P1

#define _PXOUT(port) P##port##OUT
#define PXOUT(port) _PXOUT(port)

//Size of input buffer
#define UART_RX_BUFFER_SIZE    16
//Set to 0 to use the stack-style input buffer, instead.
#define USE_RX_RINGBUFFER       1


/*----------------------------------------------------------------------------*/
// Don't edit below this.
#if USE_LEDS_TXRX

#define LED_SET(led_pin, led_val) \
  do { P1OUT = ((led_val) != 0) ? (P1OUT | (1<<(led_pin))) : (P1OUT & ~(1<<(led_pin))); } while(0)

#define SETUP_UART_LEDS()  \
  do { \
      P1OUT &= ~((1 << LED_TX) | (1 << LED_RX)); \
      P1DIR |= ((1 << LED_TX) | (1 << LED_RX)); \
  } while (0)

#else

#define SETUP_UART_LEDS()
#define LED_SET(led_pin, val)
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

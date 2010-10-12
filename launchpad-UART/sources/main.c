#include <io430.h>
#include <in430.h>
#include "UART.h"

#define UART_TEST_PATTERN "\xAA\x55\xAA\x55\xAA\x55\xAA\x55\x00"


void init_clock(void)
{
  // Internal calibrated oscillator @ 1MHz.
  BCSCTL1 = CALBC1_1MHZ;		// Set range
  DCOCTL = CALDCO_1MHZ;			// SMCLK = DCO = 1MHz   
}


void main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  init_clock();
  initUART();
  
  __enable_interrupt();
    
  while(1) {
    
    UART_echo_mode();
    /*
    UART_tx_string(UART_TEST_PATTERN);
    __delay_cycles(500000uL);
    */
  }
}

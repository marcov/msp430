#include <io430.h>
#include <in430.h>
#include "UART.h"

void main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  BCSCTL1 = CALBC1_1MHZ;		// Set range
  DCOCTL = CALDCO_1MHZ;			// SMCLK = DCO = 1MHz  
  
  initUART();

  
  __enable_interrupt();
    

  
  while(1) {
    
    //UART_echo_mode();
    
    UART_tx_string("\xAA\x55\xAA\x55\xAA\x55\xAA\x55\x00");
    
    for(unsigned int i = 0; i<0xFFFF; i++);
  }
}

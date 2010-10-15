#include <io430.h>
#include <in430.h>
#include "UART.h"
#include "board_support.h"

#define UART_TEST_PATTERN "\xAA\x55\xAA\x55\xAA\x55\xAA\x55\x00"

static unsigned char application_mode=0;

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
  
  P1SEL |= BIT4;    //Enable SMCLK out
  P1DIR |= BIT4;
  
  // Set pin of push button as input with interrupt capability on rising edge.
  P1SEL &= ~PUSH_BUTTON_PIN;
  P1DIR &= ~PUSH_BUTTON_PIN;
  P1DIR &= ~PUSH_BUTTON_PIN;
  P1IFG &= ~PUSH_BUTTON_PIN;
  P1IE |= PUSH_BUTTON_PIN;
  
  __enable_interrupt();
    
  while(1) {
    
    if (application_mode == 0) {
      UART_echo_mode();
    } else {
      UART_tx_string(UART_TEST_PATTERN);
      __delay_cycles(500000uL);
    }
  }
}



/*----------------------------------------------------------------------------*/
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  if (P1IFG & RXD) {
    start_UART_rx();
  } else if (P1IFG & PUSH_BUTTON_PIN) {
    P1IFG &= ~PUSH_BUTTON_PIN;
    application_mode ^= 1;
    __low_power_mode_off_on_exit();
  }
  
}
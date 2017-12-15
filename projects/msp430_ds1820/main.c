#if defined(__GNUC__)
#include <msp430.h>
#else
#include <io430.h>
#include <in430.h>
#endif
#include <stdbool.h>
#include "UART.h"
#include "board_support.h"
#include "ds1820.h"
#include "shared.h"
#include "ADC10.h"
#include "soft_delay.h"


bool mode_interactive = true;
static unsigned char uartbuff[10];

void init_clock(void)
{
  // Internal calibrated oscillator @ 1MHz.
  /*
  BCSCTL1 = CALBC1_1MHZ;		// Set range
  DCOCTL = CALDCO_1MHZ;			// SMCLK = DCO = 1MHz
  */

  //Self calibrated 1.00 MHz
  BCSCTL1 = XT2OFF + (RSEL0 + RSEL1 + RSEL2);
  DCOCTL  = (MOD0 + MOD1 + MOD2 + MOD3 + MOD4) + (DCO0); //modulator off.


  //Self  calibrated 2.00 MHz
  /*
  BCSCTL1 = XT2OFF + (RSEL3);
  DCOCTL  = (MOD0 + MOD1 + MOD3) + (DCO1 + DCO2);
  */

  //Self  calibrated 4.00 MHz
  /*
  BCSCTL1 = XT2OFF + (RSEL3 + RSEL1);
  DCOCTL  = (MOD3) + (DCO1 + DCO2);
  */
}


int main(void)
{
  unsigned short msp430_temperature;
  signed short ds1820_temperature;

  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  // Set 32ms WDT and clear it.
  //WDTCTL = WDT_MRST_32;

  init_clock();
  initUART();
  initADC10();

  P1SEL |= BIT4;    //Enable SMCLK out
  P1DIR |= BIT4;

  // Set pin of push button as input with interrupt capability on rising edge.
  P1SEL &= ~PUSH_BUTTON_PIN;
  P1DIR &= ~PUSH_BUTTON_PIN;
  P1DIR &= ~PUSH_BUTTON_PIN;
  P1IFG &= ~PUSH_BUTTON_PIN;
  P1IE |= PUSH_BUTTON_PIN;

  __nop();
  __enable_interrupt();

  //UART_putch('H');
  UART_puts((unsigned char *)"HELO");

  while(1)
  {
    unsigned char rxchar;

    UART_puts((unsigned char *)"\r\n");

    //WDTCTL = WDT_MRST_32;
    if (mode_interactive)    rxchar = UART_getch();
    else                     rxchar = 'x';

    if (rxchar == 'x') {
        // Perform a single conversion of DS1820 and return the registers.
        unsigned char txbuffer[DS1820_SPAD_REGISTER_SIZE];
        DS1820_get_temperature_registers(txbuffer);
        UART_putbuffer(txbuffer,sizeof(txbuffer));

    } else if (rxchar == 'd') {

        // Perform a single conversion of DS1820 and return converted value
        DS1820_get_temperature(&ds1820_temperature);
        u16_to_5digits_str(ds1820_temperature, (char *)uartbuff);
        uartbuff[5] = uartbuff[4];
        uartbuff[4] = uartbuff[3];
        uartbuff[3] = '.';
        uartbuff[6] = '\0';
        UART_puts(uartbuff);

    }else if (rxchar == 'm') {
        unsigned char txbuffer[sizeof(unsigned int)];
        msp430_temperature = get_MSP430_temperature_digital();
        txbuffer[0] = (msp430_temperature & 0x00FF);
        txbuffer[1] = (msp430_temperature>>8);
        UART_putbuffer(txbuffer,sizeof(txbuffer));
    }else if (rxchar == 'n') {
        long t;
        t = get_MSP430_temperature_digital();
        u16_to_5digits_str((unsigned)t, (char *)uartbuff);
        uartbuff[5] = '\0';
        UART_puts(uartbuff);

    } else {
        UART_putch(rxchar);
    }
  }
}



/*----------------------------------------------------------------------------*/
// Port 1 interrupt service routine
#if !defined(__GNUC__)
#pragma vector=PORT1_VECTOR
__interrupt
#else
__attribute__ ((interrupt(PORT1_VECTOR)))
#endif
void PORT1_ISR(void)
{
  if (P1IFG & PUSH_BUTTON_PIN) {
    P1IFG &= ~PUSH_BUTTON_PIN;
    mode_interactive = !mode_interactive;
    __low_power_mode_off_on_exit();
  }

}

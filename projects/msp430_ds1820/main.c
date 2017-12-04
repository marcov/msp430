#include <io430.h>
#include <in430.h>
#include "UART.h"
#include "board_support.h"
#include "DS1820.h"
#include "shared.h"
#include "ADC10.h"
#include "soft_delay.h"


static unsigned char application_mode=0;


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


void main( void )
{
  unsigned short msp430_temperature;
  signed short ds1820_temperature;
  unsigned char t_conversion_buffer [] = {'0', '0', '0', '0', '0', '0','\0'};

  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

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
  
  __enable_interrupt();
    
  while(1) {
    
    if (application_mode == 0) {
      unsigned char rxchar = UART_getch();
      
      if (rxchar == 'x') {
        // Perform a single conversion of DS1820 and return the registers.     
        unsigned char txbuffer[DS1820_SPAD_REGISTER_SIZE];
        DS1820_get_temperature_registers(txbuffer);
        UART_putbuffer(txbuffer,sizeof(txbuffer));
      
      } else if (rxchar == 'd') {
      
        // Perform a single conversion of DS1820 and return converted value
        DS1820_get_temperature(&ds1820_temperature);
        u16_to_5digits_str(ds1820_temperature, (char *)t_conversion_buffer);
        t_conversion_buffer[5] = t_conversion_buffer[4];
        t_conversion_buffer[4] = t_conversion_buffer[3];
        t_conversion_buffer[3] = '.';
        UART_puts(t_conversion_buffer);    
      
      }else if (rxchar == 'm') {
        unsigned char txbuffer[sizeof(unsigned int)];
        msp430_temperature = get_MSP430_temperature_digital();
        txbuffer[0] = (msp430_temperature & 0x00FF);
        txbuffer[1] = (msp430_temperature>>8);
        UART_putbuffer(txbuffer,sizeof(txbuffer));
      
      }
    } else if (application_mode == 1){
      // Perform a single conversion of DS1820 and publish it.
      DS1820_get_temperature(&ds1820_temperature);
      u16_to_5digits_str(ds1820_temperature, (char *)t_conversion_buffer);
      t_conversion_buffer[5] = t_conversion_buffer[4];
      t_conversion_buffer[4] = t_conversion_buffer[3];
      t_conversion_buffer[3] = '.';
      UART_puts(t_conversion_buffer);    
      UART_puts("\x0d\x0a\0");
    }
  }
}



/*----------------------------------------------------------------------------*/
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  if (P1IFG & PUSH_BUTTON_PIN) {
    P1IFG &= ~PUSH_BUTTON_PIN;
    application_mode = (application_mode+1)&0x01;
    __low_power_mode_off_on_exit();
  }
  
}
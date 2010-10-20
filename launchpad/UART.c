/******************************************************************************
 * Half Duplex Software UART on the LaunchPad
 * This implementation drives TXD as Capture/Compare, accuracy should be higher
 * than the GPIO version at high bitrates.
 *
 *  UART Basics
 *  TX and RX transmit order:
 *
 *   (1st bit)                     (last bit)
 *   +-------+-------+-----+-------+-------+
 *   | Start | 0 LSb | ... | 7 MSb | Stop  |
 *   +-------+-------+-----+-------+-------+
 *
 ******************************************************************************
 *
 * Description: UART Library.
 * Author: Marco Vedovati
 *
 ******************************************************************************
 * Original implementation from: Nicholas J. Conn - http://msp430launchpad.com
 ******************************************************************************/
#include <io430.h>
#include <in430.h>
#include "UART.h"             //Here the definition of UART baudrate.
#include "board_support.h"    //Here the definition of pinout.


//Do a more precise rounding.
#define ONE_BIT_TIME    ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 10)
#define HALF_BIT_TIME   ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 20)


/*----------------------------------------------------------------------------*/

typedef enum {
  UART_IDLE,
  UART_RECEIVING,
  UART_RECEIVED,
  UART_ERROR,
  UART_TRANSMITTING
} UART_mode_t;

/*
 * Variables declaration
 */
static unsigned char UART_bits_ctr;	// Bit count, used when transmitting byte
static unsigned short UART_tx_char;	// Value sent over UART when UART_tx_char() is called
static unsigned short UART_rx_char;	// Value recieved once hasRecieved is set
static UART_mode_t UART_mode;

/*----------------------------------------------------------------------------*/
/*
 * Fxs prototypes
 */
static void start_UART_tx_char(char chr);

/*----------------------------------------------------------------------------*/
/*
 * \brief Initializes the UART pins.
 * \param
 * \return 
 *
 *
 */
void initUART(void)
{
  //P1SEL |= TXD;     // For compare mode.
  P1OUT |= TXD;     // Set TX as idle high.
  P1DIR |= (TXD+BIT0);
  
  //RXD Pin
  P1OUT |= RXD;
  P1SEL |= RXD;
  P1DIR &= ~RXD;
  
  //CM = 2 : Capture on falling edge
  // CCIS = 0 : Capture input select input signal A
  // CAP : Capture mode
  //TACCTL1 = CM_2 + CCIS_1 + CAP;
  TACCTL1 = CM_2 + CCIS_0 + CAP;

  // TASSEL = 2 : Clock source SMCLK
  // MC = 2 : Start timer in continuous mode.
  TACTL = TASSEL_2 + MC_2;
  
  TACCTL1 |= CCIE;  //Enable interrupt;
  
  UART_mode = UART_IDLE;
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Echo back character received from UART.
 * \param
 * \return 
 *
 *
 */
void UART_echo_mode(void)
{
  if (UART_mode == UART_RECEIVED) {
    // If the device has recieved a value
    UART_mode = UART_IDLE;
    start_UART_tx_char(UART_rx_char);
  } else {
    __low_power_mode_0();        
    // LPM0, the PORT1 interrupt will wake the processor up. This is so that it does not
    //	endlessly loop when no value has been Received.
  }
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Returns a character from UART.
 *
 * This is a blocking function: it waits for a character 
 * from UART (if not already received).
 *
 * \param
 * \return the character receved from UART
 *
 *
 */
unsigned char UART_get_char(void)
{
  while (UART_mode != UART_RECEIVED) {
    __low_power_mode_0();
  }
  
  UART_mode = UART_IDLE;
  return UART_rx_char;
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Transmits a string over UART.
 * \param str The string to transmit, terminated by a '\0' character.
 * \return 
 *
 *
 */
void UART_tx_string (char * str)
{
  while ( *str != '\0' ) {
    start_UART_tx_char(*str);
    __low_power_mode_0(); //go to low power until tx has finished, but keep SMCLK on.
    str++;
  }
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Trigger the start of a char TX over UART.
 * \param the character to transmit.
 * \return 
 *
 *
 */
static void start_UART_tx_char(char chr)
{ 
  while(UART_mode == UART_RECEIVING);   // Wait for RX completion
  
  UART_tx_char = chr;	                // Load the recieved byte into the byte to be transmitted
  UART_tx_char |= 0x100;                // Add stop bit to UART_tx_char (which is logical 1)
  UART_bits_ctr = 9;			// Load Bit counter, 8 bits + STOP. We send start now.
  
  UART_mode = UART_TRANSMITTING;
  
  P1SEL |= TXD;
  
  
  TACCTL0 = OUTMOD_5;                // will set output low for start bit.    
  
  TACCR0 = TAR+HALF_BIT_TIME;   //Delay the start bit of a little time.
  
  TACCTL0 |= CCIE;          // Interrupt should fire immediately.
  
  ACT_LED = 1;
}


/*----------------------------------------------------------------------------*/
/*
 * \brief Function called when the Start bit of a RX'd char is detected.
 *
 * You may want to call this function as irq handler for RXD falling edge.
 * \param
 * \return 
 *
 *
 */
void start_UART_rx(void)
{
  //Start of RX
  UART_mode = UART_RECEIVING;
  ACT_LED = 1;
  
  TACCTL1 &= ~(CCIE+CM_3);
  TACCR0 = CCR1 + HALF_BIT_TIME;
  TACCTL0 = OUT + CCIE;   // Setting out is necessary to keep the TX Line high.      	  
  
  UART_rx_char = 0x00;		  // Initialize UART_rx_char
  UART_bits_ctr = 10;		  // Load Bit counter, 8 bits + STOP    
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{
  if(UART_mode == UART_TRANSMITTING) {

    if (UART_bits_ctr > 0) {
      // Still bits 2 TX
      if (UART_tx_char & 0x01) {
        //P1OUT |= TXD;
        TACCTL0 &= ~OUTMOD_4;
      }
      else {
        //P1OUT &= ~TXD;
        TACCTL0 |= OUTMOD_5;
      }
      
      TACCR0 += ONE_BIT_TIME;		// Add Offset to CCR0. 
      UART_tx_char >>= 1;
      UART_bits_ctr--;
    } else {	
       // If all bits TXed
      CCTL0 &= ~CCIE ;		// Disable interrupt
      P1SEL &= ~TXD;
 
      UART_mode = UART_IDLE;
      ACT_LED =0;
      __low_power_mode_off_on_exit();      
    }
  } if(UART_mode == UART_RECEIVING) {

    if ( (P1IN & RXD) == RXD) {		
      // If bit is set?
      UART_rx_char |= 0x0400;		// Set the value in the UART_rx_char
    }
    UART_rx_char >>= 1;		        // Shift the bits down
    UART_bits_ctr --;
    
    if (UART_bits_ctr > 0) {
      TACCR0 += ONE_BIT_TIME;		// Add Offset to CCR0.
    } else {
      // All byte received.
      TACCTL0 &= ~ CCIE ;		// Disable interrupt for compare
      TACCTL1 |= (CCIE+ CM_2);          // Restore capture capability
      
      if ( (UART_rx_char & 0x201) == 0x200) {
        // Validate the start and stop bits are correct
        
        UART_rx_char = UART_rx_char >> 1;	// Remove start bit
        UART_rx_char &= 0xFF;			// Remove stop bit
        UART_mode = UART_RECEIVED;
      } else {
        UART_mode = UART_ERROR;
      }
      
      ACT_LED = 0;
      __low_power_mode_off_on_exit();
    }
  }
  
}



/*----------------------------------------------------------------------------*/
// Timer A CCR1 interrupt service routine
#pragma vector=TIMERA1_VECTOR
__interrupt void TimerA1_ISR (void)
{
  TACCTL1 &= ~CCIFG;
  // So, we have detected a start bit falling edge. We need to sample it again
  // in HALF_BIT_TIME, this time using compare register.
  start_UART_rx();
  __no_operation();
}

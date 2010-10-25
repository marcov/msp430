/******************************************************************************
 * Half Duplex Software UART on the LaunchPad
 * This implementation drives TXD as by using Timer Compare and set RXD sampling 
 * by using the Timer Capture, hence accuracy should be higher than GPIO verison.
 *
 * You must supply a valid board_support.h file with:
 * - definition of TXD and RXD pins (bound to CCR module assignment).
 * - definition of your SMCLK_FREQUENCY (Timer is sourced from SMCLK).
 * 
 *
 *  UART Basics
 *  TX and RX transmit order:
 *
 *   (1st bit)                     (last bit)
 *   +-------+-------+-----+-------+-------+
 *   | Start | 0 LSb | ... | 7 MSb | Stop  |
 *   +-------+-------+-----+-------+-------+
 *
 *----------------------------------
 *
 * Module Description: UART Library.
 * Author: Marco Vedovati
 *
 *------------------------------------------------------------------------------
 * Original implementation from: Nicholas J. Conn - http://msp430launchpad.com
 ******************************************************************************/
#include <io430.h>
#include <in430.h>
#include "UART.h"             //Here the definition of UART baudrate.
#include "board_support.h"
                              

//TODO: Add ring buffer.
//      Port to Full Duplex.
//      For half duplex a single CCR module should be enough.


//Do a more precise rounding by using the ((..)*10 +5) /10 trick.
#define ONE_BIT_TIME    ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 10)
#define HALF_BIT_TIME   ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 20)

#define UART_BUFFER_MAX_SIZE  10
/*----------------------------------------------------------------------------*/

typedef enum {
  UART_IDLE         = 0x00,
  UART_RECEIVING    = 0x01,
  UART_TRANSMITTING = 0x02,
} UART_mode_t;

/*
 * Variables declaration
 */
static unsigned char UART_bits_ctr;	// Bit count, used when transmitting byte
static unsigned short UART_tx_char;	// Value sent over UART when UART_tx_char() is called
static unsigned short UART_rx_char;	// Value recieved once hasRecieved is set
static UART_mode_t UART_mode;

static unsigned char UART_buffer[UART_BUFFER_MAX_SIZE];   // This is a stack type UART buffer shared by RX and TX
static unsigned char UART_buffer_idx = 0; // This is the index for the previous array.

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
  //TXD Section
  P1DIR |= (TXD);
  P1SEL |= TXD;     //Compare Peripheral. Set OUT of compare module for IDLE HIGH.
  
  // OUT = 1 : Output high when OUTMODE = 0;
  TACCTL0 = OUT;    
  
  //RXD Section
  P1OUT |= RXD;
  P1SEL |= RXD;
  P1DIR &= ~RXD;
      
  // CM = 2 : Capture on falling edge
  // CCIS = 0 : Capture input select input signal A
  // CAP : Capture mode
  TACCTL1 = CM_2 + CCIS_0 + CAP;
  TACCTL1 |= CCIE;  //Enable interrupt;

  // TASSEL = 2 : Clock source SMCLK
  // MC = 2 : Start timer in continuous mode.
  TACTL = TASSEL_2 + MC_2;
  
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
  unsigned char tmp_char;
  
  if (UART_buffer_idx > 0) {
    // If the device has recieved a value
    __disable_interrupt();
    UART_buffer_idx--;
    tmp_char= UART_buffer[UART_buffer_idx];
    __enable_interrupt();
  
    start_UART_tx_char(tmp_char);
  }
  
  __low_power_mode_0();
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
  while (UART_buffer_idx == 0) {
    __low_power_mode_0();
  }
  
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
  UART_tx_char = chr;	                // Load the recieved byte into the byte to be transmitted
  UART_tx_char |= 0x100;                // Add stop bit to UART_tx_char (which is logical 1)
  UART_bits_ctr = 9;			// Load Bit counter, 8 bits + STOP. We send start now.
                        
  TACCR0 = TAR+HALF_BIT_TIME;           //Delay the start bit of a little time.
  TACCTL0 &= ~CCIFG;                                      
  TACCTL0 |= (OUTMOD_5 + CCIE);         // will set output low for start bit.
                                        // Interrupt should fire immediately.

  UART_mode |= UART_TRANSMITTING;
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
  UART_rx_char = 0x00;		  // Initialize UART_rx_char
  UART_bits_ctr = 10;		  // Load Bit counter, 8 bits + STOP 
  
  TACCR1 += HALF_BIT_TIME;     	  // set next compare value.
  TACCTL1 &= ~(CM_3+CAP);         // Disable capture mode -> set compare mode.
  
  UART_mode |= UART_RECEIVING;
  ACT_LED = 1;
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{
  TACCTL0 &= ~CCIFG;
  if(UART_mode & UART_TRANSMITTING) {

    if (UART_bits_ctr > 0) {
      // Still bits 2 TX
      if (UART_tx_char & 0x01) {
        TACCTL0 &= ~OUTMOD_4;
      }
      else {
        TACCTL0 |= OUTMOD_5;
      }
      
      TACCR0 += ONE_BIT_TIME;		// Add Offset to CCR0. 
      UART_tx_char >>= 1;
      UART_bits_ctr--;
    } else {	
       // If all bits TXed
      TACCTL0 &= ~CCIE ;		// Disable interrupt
 
      UART_mode &= ~UART_TRANSMITTING;
      ACT_LED =0;
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
  if(UART_mode & UART_RECEIVING) {

    if ( (P1IN & RXD) == RXD) {		
      // If bit is set?
      UART_rx_char |= 0x0400;		// Set the value in the UART_rx_char
    }
    UART_rx_char >>= 1;		        // Shift the bits down
    UART_bits_ctr --;
    
    if (UART_bits_ctr > 0) {
      TACCR1 += ONE_BIT_TIME;		// Add Offset to CCR0.
    } else {
      // All byte received.
      TACCTL1 |= (CCIE + CM_2 + CAP);   // Restore capture capability
      
      if ( (UART_rx_char & 0x201) == 0x200) {
        // Validate the start and stop bits are correct
        
        UART_rx_char = UART_rx_char >> 1;	// Remove start bit
        UART_rx_char &= 0xFF;			// Remove stop bit
        
        __disable_interrupt();
        if (  (UART_buffer_idx < UART_BUFFER_MAX_SIZE) ) {
          // Now char has been received, so we can stack in on
          UART_buffer[UART_buffer_idx++] = UART_rx_char;
        }
        __enable_interrupt();
      } else {
        //TODO: set ERROR received.
      }
      
      UART_mode &= ~UART_RECEIVING;
      ACT_LED = 0;
      __low_power_mode_off_on_exit();
    }
  } else {
    // So, we have detected a start bit falling edge. We need to sample it again
    // in HALF_BIT_TIME, this time using compare register.
    start_UART_rx(); 
  }
}

/******************************************************************************
 * UART Library for the MSP430 Launchpad
 *
 * Author:   Marco Vedovati
 * Email :   msp430@vedovati.info
 * Compiler: IAR EW MSP430 5.10.6
 *
 * Description:
 * Half-duplex software-UART for the MSP430 LaunchPad.
 * This implementation uses two TimerA CC modules:
 * - TXD uses of TACCR0 module in compare mode only;
 * - RXD uses TACCR1 in capture mode to sample the START bit and reads
 *   the following using the compare mode.
 * Capture/compare are used to obtain a higher accuracy than a GPIO
 * implementation. 
 *
 * To use this library, you must supply a valid "board_support.h" file with:
 * - definition of TXD and RXD pins: they are bound to TACCR0-1 location, as 
 *   described earlier. Hint: TACCRx are remapped on more pins, see datasheet.
 * - definition of your SMCLK_FREQUENCY in Hz(Timer is sourced from SMCLK).
 * E.g.:
 *
 * #define TXD             BIT1                // TXD on P1.1
 * #define RXD             BIT2                // RXD on P1.2
 * #define SMCLK_FREQUENCY 1000000
 *
 * UART baudrate and LEDs signaling for TX/RX can be set in UART.h
 *------------------------------------------------------------------------------
 * Original implementation from: Nicholas J. Conn - http://msp430launchpad.com
 ******************************************************************************/
/*----------------------------------
 *  UART Basic theory
 *  TX and RX transmit order:
 *
 *   (1st bit)                     (last bit)
 *   +-------+-------+-----+-------+-------+
 *   | Start | 0 LSb | ... | 7 MSb | Stop  |
 *   +-------+-------+-----+-------+-------+
 *
 ----------------------------------*/
#include <io430.h>
#include <in430.h>
#include "UART.h"             //Here the definition of UART baudrate.
#include "board_support.h"
                              

//TODO: Add a ring buffer (now it is just a useless stack-type buffer).
//      Port to Full Duplex.
//      For half duplex a single CCR module may be enough.


//Do a more precise rounding by using the ((..)*10 +5) /10 trick.
#define ONE_BIT_TIME    ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 10)
#define HALF_BIT_TIME   ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 20)

#define UART_BUFFER_MAX_SIZE  16
/*----------------------------------------------------------------------------*/

typedef enum {
  UART_IDLE         = 0x00,
  UART_RECEIVING    = 0x01,
  UART_TRANSMITTING = 0x02,
} UART_state_t;

/*
 * Variables declaration
 */
static unsigned char UART_bits_ctr;	// bit counter for the UART RX/TX byte.
static unsigned short UART_tx_char;	// Value to transmit / in transmission
static unsigned short UART_rx_char;	// Value received / in reception.
static UART_state_t UART_state;

static unsigned char UART_buffer[UART_BUFFER_MAX_SIZE];   // This is a stack type UART buffer shared by RX and TX
static unsigned char UART_buffer_idx = 0; // This is the index for the previous array.

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
  SETUP_UART_LEDS();
  
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
  
  UART_state = UART_IDLE;
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Echo back character received from UART.
 * You may mostly use this for testing purpose.
 * 
 * \param
 * \return 
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
  
    UART_putch(tmp_char);
  }
  
  __low_power_mode_0();
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Returns a character from UART.
 *
 * This is a blocking function: it waits for a character 
 * from UART if it has not already received.
 *
 * \param
 * \return the character receved from UART
 *
 *
 */
unsigned char UART_getch(void)
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
void UART_puts (unsigned char * str)
{
  while ( *str != '\0' ) {
    UART_putch(*str);
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
void UART_putch(unsigned char chr)
{ 
  while(UART_state & UART_TRANSMITTING); // Wait for the current TX to terminate.
  
  UART_tx_char = chr;	                // Load the recieved byte into the byte to be transmitted
  UART_tx_char |= 0x100;                // Add stop bit to UART_tx_char (which is logical 1)
  UART_bits_ctr = 9;			// Load Bit counter, 8 bits + STOP. We send start now.
                        
  TACCR0 = TAR+HALF_BIT_TIME;           //Delay the start bit of a little time.
  TACCTL0 &= ~CCIFG;                                      
  TACCTL0 |= (OUTMOD_5 + CCIE);         // will set output low for start bit.
                                        // Interrupt should fire immediately.

  UART_state |= UART_TRANSMITTING;
  LED_TX = 1;
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
  
  UART_state |= UART_RECEIVING;
  LED_RX = 1;
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{
  TACCTL0 &= ~CCIFG;
  if(UART_state & UART_TRANSMITTING) {

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
 
      UART_state &= ~UART_TRANSMITTING;
      LED_TX =0;
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
  if(UART_state & UART_RECEIVING) {

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
      
      UART_state &= ~UART_RECEIVING;
      LED_RX = 0;
      __low_power_mode_off_on_exit();
    }
  } else {
    // So, we have detected a start bit falling edge. We need to sample it again
    // in HALF_BIT_TIME, this time using compare register.
    start_UART_rx(); 
  }
}

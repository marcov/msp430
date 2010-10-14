/******************************************************************************
 * Half Duplex Software UART on the LaunchPad
 * This implementation drives TXD as GPIO, so accuracy may not be optimal.
 * An alternative implementation could use the Capture/Compare functionality.
 *
 ******************************************************************************
 * Description: UART Library.
 * Author: Marco Vedovati
 ******************************************************************************
 * Original implementation from:
 * Author: Nicholas J. Conn - http://msp430launchpad.com
 ******************************************************************************/
#include <io430.h>
#include <in430.h>
#include "UART.h"
#include "board_support.h"
/* 
 *  TX and RX transmit order:
 *
 *   (1st bit)                     (last bit)
 *   +-------+-------+-----+-------+-------+
 *   | Start | 0 LSb | ... | 7 MSb | Stop  |
 *   +-------+-------+-----+-------+-------+
 *
 */

//Baudrate = SMCLK freq(1MHz) / Baudrate
// Seems like max speed working on the Launchpad board is 9600 bauds...
#define BAUDRATE_1200    	833
#define BAUDRATE_2400    	417
#define BAUDRATE_4800    	208
#define BAUDRATE_9600    	104		

#define ONE_BIT_TIME      BAUDRATE_9600
#define HALF_BIT_TIME   (ONE_BIT_TIME / 2)

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
  P1SEL &= ~(TXD+BIT0);
  P1OUT |= TXD;     // Set TX as idle high.
  P1DIR |= (TXD+BIT0);
  
  P1IES |= RXD;				// RXD Hi/lo edge interrupt
  P1IFG &= ~RXD;			// Clear RXD (flag) before enabling interrupt
  P1IE |= RXD;				// Enable RXD interrupt
 
  TACTL = TASSEL_2;                    // TimerA clock source is SMCLK.
  
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
  UART_tx_char <<= 1;	                // Add start bit (which is logical 0)  
  UART_bits_ctr = 10;			// Load Bit counter, 8 bits + STOP. We send start now.
  
  UART_mode = UART_TRANSMITTING;
  
  TACTL |= MC_2;		        // Start TIMER continuous mode
  CCR0 = (TAR);			        // Initialize compare register
  CCTL0 = CCIE+CCIFG;
  
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
  
  P1IE &= ~RXD;			// Disable RXD interrupt
  P1IFG &= ~RXD;		// Clear RXD IFG (interrupt flag)
  
  TACTL |= MC_2;		        // Start TIMER continuous mode 
  CCR0 = TAR;			// Initialize compare register
  CCR0 += HALF_BIT_TIME;	// Latch bit at half of the bit time.
  CCTL0 = CCIE;         	// Dissable TX and enable interrupts
  
  UART_rx_char = 0;		// Initialize UART_rx_char
  UART_bits_ctr = 9;		// Load Bit counter, 8 bits + STOP    
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{                        
  if(UART_mode == UART_TRANSMITTING) {
    CCR0 += ONE_BIT_TIME;			// Add Offset to CCR0  
    
    if ( UART_bits_ctr == 0) {		
      // If all bits TXed
      UART_mode = UART_IDLE;
      
      TACTL_bit.TAMC &= 0;	// timer off (for power consumption)
      CCTL0 &= ~ CCIE ;		// Disable interrupt
      __low_power_mode_off_on_exit();
      ACT_LED =0;
    }
    else {
      if (UART_tx_char & 0x01) {
        P1OUT |= TXD;
      }
      else {
        P1OUT &= ~TXD;
      }
      UART_tx_char >>= 1;
      UART_bits_ctr--;
    }
  } else {
    CCR0 += ONE_BIT_TIME;				// Add Offset to CCR0  
    if ( UART_bits_ctr == 0) {
      TACTL_bit.TAMC &= 0;	// timer off (for power consumption)
      CCTL0 &= ~ CCIE ;			// Disable interrupt
      
      P1IFG &= ~RXD;				// clear RXD IFG (interrupt flag)
      P1IE |= RXD;				// enabled RXD interrupt
      
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
    else {
      if ( (P1IN & RXD) == RXD) {		
        // If bit is set?
        UART_rx_char |= 0x400;		// Set the value in the UART_rx_char
      }
      UART_rx_char >>= 1;		// Shift the bits down
      UART_bits_ctr --;
    }
  }
}

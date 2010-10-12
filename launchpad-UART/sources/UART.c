/******************************************************************************
 *                 Half Duplex Software UART on the LaunchPad
 *
 ******************************************************************************
 * Description: UART Library.
 * Author: Marco Vedovati
 *
 ******************************************************************************
 * Initial implementation from:
 * Author: Nicholas J. Conn - http://msp430launchpad.com
 ******************************************************************************/
#include <io430.h>
#include <in430.h>

#define TXD         BIT1                // TXD on P1.1
#define RXD         BIT2                // RXD on P1.2
#define ACT_LED     P1OUT_bit.P1OUT_0   // Activity led out port


//Baudrate = SMCLK freq(1MHz) / Baudrate
// Seems like max speed working is 9600 bauds...
#define BAUDRATE_4800    	208
#define BAUDRATE_9600    	104		

#define ONE_BIT_TIME      BAUDRATE_9600
#define CENTER_BIT_TIME   (ONE_BIT_TIME / 2)


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
static unsigned char UART_bits_ctr;		// Bit count, used when transmitting byte
static unsigned short UART_tx_char;	// Value sent over UART when UART_tx_char() is called
static unsigned short UART_rx_char;	// Value recieved once hasRecieved is set

static UART_mode_t UART_mode;

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
  P1DIR |= (TXD+BIT0);
  
  P1IES |= RXD;				// RXD Hi/lo edge interrupt
  P1IFG &= ~RXD;			// Clear RXD (flag) before enabling interrupt
  P1IE |= RXD;				// Enable RXD interrupt
  
  
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
 * \param
 * \return 
 *
 *
 */
static void start_UART_tx_char(char chr)
{ 
  while(UART_mode == UART_RECEIVING);   // Wait for RX completion
  
  CCTL0 = OUT;				// TXD Idle as Mark
  TACTL = TASSEL_2 + MC_2;		// SMCLK, continuous mode
  
  UART_bits_ctr = 10;				// Load Bit counter, 8 bits + ST/SP
  CCR0 = TAR;				// Initialize compare register
  
  UART_tx_char = chr;	                // Load the recieved byte into the byte to be transmitted
  
  CCR0 += ONE_BIT_TIME;		// Set time till first bit
  UART_tx_char |= 0x100;		// Add stop bit to UART_tx_char (which is logical 1)
  UART_tx_char <<= 1;	                // Add start bit (which is logical 0)
  
  UART_mode = UART_TRANSMITTING;
  
  CCTL0 = CCIE;
  ACT_LED = 1;
}


/*----------------------------------------------------------------------------*/
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{  	
  UART_mode = UART_RECEIVING;
  
  ACT_LED = 1;
  
  P1IE &= ~RXD;			// Disable RXD interrupt
  P1IFG &= ~RXD;		// Clear RXD IFG (interrupt flag)
  
  TACTL = TASSEL_2 + MC_2;	// SMCLK, continuous mode
  CCR0 = TAR;			// Initialize compare register
  CCR0 += CENTER_BIT_TIME;	// Set time till first bit
  CCTL0 = OUTMOD_1 + CCIE;	// Dissable TX and enable interrupts
  
  UART_rx_char = 0;		// Initialize UART_rx_char
  UART_bits_ctr = 0x9;		// Load Bit counter, 8 bits + ST
  
  
}

/*----------------------------------------------------------------------------*/
// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{
  if(UART_mode == UART_TRANSMITTING) {
    CCR0 += ONE_BIT_TIME;			// Add Offset to CCR0  
    
    if ( UART_bits_ctr == 0) {		
      // If all bits TXed
      UART_mode = UART_IDLE;
      
      TACTL = TASSEL_2;		// SMCLK, timer off (for power consumption)
      CCTL0 &= ~ CCIE ;		// Disable interrupt
      __low_power_mode_off_on_exit();
      ACT_LED =0;
    }
    else {
      if (UART_tx_char & 0x01) {
        //CCTL0 &= ~ OUTMOD_2;	   // If it should be 1, set it to 1
        P1OUT |= TXD;
      }
      else {
        //CCTL0 |=  OUTMOD_2;	   // Set TX bit to 0
        P1OUT &= ~TXD;
      }
      UART_tx_char = UART_tx_char >> 1;
      UART_bits_ctr --;
    }
  } else {
    CCR0 += ONE_BIT_TIME;				// Add Offset to CCR0  
    if ( UART_bits_ctr == 0) {
      TACTL = TASSEL_2;			// SMCLK, timer off (for power consumption)
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
    else
    {
      if ( (P1IN & RXD) == RXD) {		
        // If bit is set?
        UART_rx_char |= 0x400;		// Set the value in the UART_rx_char
      }
      UART_rx_char >>= 1;		// Shift the bits down
      UART_bits_ctr --;
    }
  }
}

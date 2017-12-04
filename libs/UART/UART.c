/******************************************************************************
 * UART Library for the MSP430 Launchpad
 *
 * Author:   Marco Vedovati
 * Email :   msp430@vedovati.info
 * Compiler: IAR EW MSP430 5.10.6
 *
 * Description:
 * FULL-duplex software-UART for the MSP430 LaunchPad.
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
 *
 * UART baudrate, LEDs signaling for TX/RX and ringbuffer size and usage
 * must be set in UART.h
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
//TODO: Branch a half-duplex version using CCR module.
#include <io430.h>
#include <in430.h>
#include "UART.h"             //Here the definition of UART baudrate.
#include "board_support.h"
                             
#if USE_RX_RINGBUFFER
#include "ringbuf.h"
#endif

//Do a more precise rounding by using the ((..)*10 +5) /10 trick.
#define ONE_BIT_TIME    ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 10)
#define HALF_BIT_TIME   ( ( (SMCLK_FREQUENCY*10 / BAUDRATE) + 5) / 20)

/*----------------------------------------------------------------------------*/

typedef enum {
  UART_IDLE         = 0x00,
  UART_RECEIVING    = 0x01,
  UART_TRANSMITTING = 0x02,
} UART_state_t;

/*
 * Variables declaration
 */
static unsigned char UART_TX_bits_ctr;	// bit counter for the UART TX byte.
static unsigned char UART_RX_bits_ctr;	// bit counter for the UART RX byte.
static unsigned short UART_tx_char;	// Value to transmit / in transmission
static unsigned short UART_rx_char;	// Value received / in reception.
static UART_state_t UART_state;

// This buffer is common for both the stack type and the ringbuffer type
// UART rx buffer. Dont use this to access to buffer, use the 
// variables defined below.
static unsigned char __UART_buffer[UART_RX_BUFFER_SIZE];

#if USE_RX_RINGBUFFER
struct ringbuf UART_rb;
#else
// Use this name to acess to the stack-style rx buffer.
#define UART_rx_buffer __UART_buffer
//Index for the stack-style UART_rx_buffer.
static unsigned char UART_rx_buffer_idx;
#endif

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
  
#if USE_RX_RINGBUFFER
  ringbuf_init(&UART_rb, __UART_buffer, UART_RX_BUFFER_SIZE);
#else
  UART_rx_buffer_idx = 0;
#endif
  
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
  // Syncronize capture source.
  TACCTL1 = CM_2 + CCIS_0 + CAP + SCS;
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
  UART_putch( UART_getch() );
/*
  signed int echo_char;
  istate_t intstate;
  
  intstate = __get_interrupt_state();
  __disable_interrupt();
  
#if USE_RX_RINGBUFFER
  echo_char = ringbuf_get(&UART_rb);
  __set_interrupt_state(intstate);
  if (echo_char != -1) {

#else  
  if (UART_rx_buffer_idx > 0) {
    UART_rx_buffer_idx--;
    echo_char= UART_rx_buffer[UART_rx_buffer_idx];
    __set_interrupt_state(intstate);

#endif    
    UART_putch((unsigned char)echo_char);
  } else {
    __low_power_mode_0();
  }
*/
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
  signed short retchar;
  
#if USE_RX_RINGBUFFER
  retchar = ringbuf_get(&UART_rb);
  while (retchar == -1) {
    __low_power_mode_0();
    retchar = ringbuf_get(&UART_rb);
  }
#else
  while (UART_rx_buffer_idx == 0) {
    __low_power_mode_0();
  }
  retchar = UART_rx_buffer[--UART_rx_buffer_idx];
#endif
  
  return retchar;
}

/*----------------------------------------------------------------------------*/
/*
 * \brief Transmits a null terminated string over UART.
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
 * \brief Transmits an array of characters over UART.
 * \param txbuffer The array to transmit.
 * \param length   The sizeo of the array.
 * \return 
 *
 *
 */
void UART_putbuffer (unsigned char * txbuffer, unsigned char length)
{
  for (unsigned char i = 0 ; i<length; i++) {
    UART_putch(txbuffer[i]);
    __low_power_mode_0(); //go to low power until tx has finished, but keep SMCLK on.
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
  UART_TX_bits_ctr = 9;			// Load Bit counter, 8 bits + STOP. We send start now.
                        
  TACCR0 = TAR+HALF_BIT_TIME;           //Delay the start bit of a little time.
  TACCTL0 &= ~CCIFG;                                      
  TACCTL0 |= (OUTMOD_5 + CCIE);         // will set output low for start bit.
                                        // Interrupt should fire immediately.

  UART_state |= UART_TRANSMITTING;
  LED_TX = 1;
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void TimerA0_ISR (void)
{
  TACCTL0 &= ~CCIFG;
  if(UART_state & UART_TRANSMITTING) {

    if (UART_TX_bits_ctr > 0) {
      // Still bits 2 TX
      if (UART_tx_char & 0x01) {
        TACCTL0 &= ~OUTMOD_4;
      }
      else {
        TACCTL0 |= OUTMOD_5;
      }
      
      TACCR0 += ONE_BIT_TIME;		// Add Offset to CCR0. 
      UART_tx_char >>= 1;
      UART_TX_bits_ctr--;
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
    if (P1IN & RXD) {		
      // If bit is set?
      UART_rx_char |= 0x0400;		// Set the value in the UART_rx_char
    }
    UART_rx_char >>= 1;		        // Shift the bits down
    UART_RX_bits_ctr--;
    
    if (UART_RX_bits_ctr > 0) {
      // Still bits left to receive.
      TACCR1 += ONE_BIT_TIME;
    } else {
      // Whole byte received.
      TACCTL1 |= (CCIE + CM_2 + CAP + SCS);   // Restore capture capability
      P1SEL |= RXD;                     // Restore peripheral mode on pin.
      
      if ( (UART_rx_char & 0x201) == 0x200) {
        // Validate the start and stop bits are correct
        
        UART_rx_char = UART_rx_char >> 1;	// Remove start bit
        UART_rx_char &= 0xFF;			// Remove stop bit

        __disable_interrupt();
#if USE_RX_RINGBUFFER
        ringbuf_put(&UART_rb, UART_rx_char);
#else
        if (  (UART_rx_buffer_idx < UART_RX_BUFFER_SIZE) ) {
          // Now char has been received, so we can stack in on
          UART_rx_buffer[UART_rx_buffer_idx++] = UART_rx_char;
        } else {
          //TODO: buffer full
        }
#endif
        __enable_interrupt();
      } else {
        //TODO: Hanlde RX error
        __no_operation();
      }
      UART_state &= ~UART_RECEIVING;
      LED_RX = 0;
      __low_power_mode_off_on_exit();
    }
    
  } else {
    // So, we have detected a start bit falling edge. We need to sample it again
    // in HALF_BIT_TIME, this time using compare register.
    UART_rx_char = 0x00;		  // Initialize UART_rx_char
    UART_RX_bits_ctr = 10;		  // Load Bit counter, 8 bits + STOP 
    
    TACCR1 += HALF_BIT_TIME;     	  // set next compare value.
    P1SEL &= ~RXD;                  // Disable peripheral mode on pin.
    TACCTL1 &= ~(CM_3+CAP);         // Disable capture mode -> set compare mode.
    
    UART_state |= UART_RECEIVING;
    LED_RX = 1;
  }
}

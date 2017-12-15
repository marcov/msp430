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
 * must be set in uart.h
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
#if defined(__GNUC__)
#include <msp430.h>
#else
#include <io430.h>
#include <in430.h>
#endif
//TODO: Branch a half-duplex version using CCR module.
#include "uart.h"             //Here the definition of UART baudrate.
#include "board_support.h"

#if USE_RX_RINGBUFFER
#include "ringbuf.h"
#endif

//Do a more precise rounding by using the ((..)*10 +5) /10 trick.
#define ONE_BIT_TA_TICKS    ( ( (SMCLK_FREQUENCY*10 / UART_BAUDRATE) + 5) / 10)
#define HALF_BIT_TA_TICKS   ( ( (SMCLK_FREQUENCY*10 / UART_BAUDRATE) + 5) / 20)			//Time expressed in TimerA clock ticks

#define MICROSECONDS_IN_SEC     1000000uL
#define ONE_BIT_TIME        ( ( (MICROSECONDS_IN_SEC*10/ UART_BAUDRATE) + 5) / 10)
#define HALF_BIT_TIME       ( ( (MICROSECONDS_IN_SEC*10/ UART_BAUDRATE) + 5) / 20)     //Time expressed in us

//Values measured sperimentally!!
#define ISR_DELAY_CYCLES    35 //This is the delay from the bit edge to the actual
                               //point in the ISR in which the RXD pin is read.
#define START_BIT_TO_CCR_ASSIGN 215

#define ISR_DELAY_TICKS     ISR_DELAY_CYCLES
#define ISR_DELAY_TIME      ((ISR_DELAY_CYCLES * 1000000) / SMCLK_FREQUENCY)			//Time expressed in us



#if ISR_DELAY_TIME >= ONE_BIT_TIME
#error "Cannot support configured baudrate at current SMCLK frequency."
#elif ISR_DELAY_TIME >= HALF_BIT_TIME
#warning "Configured baudrate at current SMCLK frequency may limit UART reliability."
#endif

/*----------------------------------------------------------------------------*/

struct
{
    volatile enum {
      UART_IDLE  = 0,
      UART_RXING = (1 << 0),
      UART_TXING = (1 << 1),
    } state;

    volatile unsigned tx_bits_ctr;	// bit counter for the UART TX byte.
    volatile unsigned rx_bits_ctr;	// bit counter for the UART RX byte.
    volatile unsigned txval;	// Value to transmit / in transmission
    volatile unsigned rxval;	// Value received / in reception.
} uart_ctxt;


struct
{
    unsigned txdone;
    unsigned txstart;
    unsigned rxdone;
    unsigned rxstart;
}dbg;

// This buffer is common for both the stack type and the ringbuffer type
// UART rx buffer. Dont use this to access to buffer, use the
// variables defined below.
static unsigned char my_rx_buffer[UART_RX_BUFFER_SIZE];

#if USE_RX_RINGBUFFER
struct ringbuf uart_ringbuffer;
#else
// Use this name to acess to the stack-style rx buffer.
#define UART_rx_buffer my_rx_buffer
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
  ringbuf_init(&uart_ringbuffer, my_rx_buffer, UART_RX_BUFFER_SIZE);
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

  uart_ctxt.state = UART_IDLE;
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
  echo_char = ringbuf_get(&uart_ringbuffer);
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
  retchar = ringbuf_get(&uart_ringbuffer);
  while (retchar == -1) {
    __low_power_mode_0();
    retchar = ringbuf_get(&uart_ringbuffer);
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
  for(; *str != '\0'; str++ )
  {
    UART_putch(*str);
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
  while ((uart_ctxt.state & UART_TXING) != 0)
  {
    __low_power_mode_0(); //go to low power until tx has finished, but keep SMCLK on.
  }

  dbg.txstart++;

  uart_ctxt.txval = chr;	                // Load the recieved byte into the byte to be transmitted
  uart_ctxt.txval |= 0x100;                // Add stop bit to uart_ctxt.txval (which is logical 1)
  uart_ctxt.tx_bits_ctr = 9;			// Load Bit counter, 8 bits + STOP. We send start now.

  uart_ctxt.state |= UART_TXING;
  LED_SET(LED_TX, 1);

  TACCR0 = TAR+HALF_BIT_TA_TICKS;       //Delay the start bit of a little time.
  TACCTL0 &= ~CCIFG;
  TACCTL0 |= (OUTMOD_5 + CCIE);         // will set output low for start bit.
}


/*----------------------------------------------------------------------------*/
// Timer A CCR0 interrupt service routine
#if !defined(__GNUC__)
#pragma vector=TIMERA0_VECTOR
__interrupt
#else
__attribute__ ((interrupt(TIMER0_A0_VECTOR)))
#endif
void TimerA0_ISR (void)
{
  TACCTL0 &= ~CCIFG;
  if ((uart_ctxt.state & UART_TXING) != 0) {

    if (uart_ctxt.tx_bits_ctr > 0) {
      // Still bits 2 TX
      if (uart_ctxt.txval & 0x01) {
        TACCTL0 &= ~OUTMOD_4;
      }
      else {
        TACCTL0 |= OUTMOD_5;
      }

      TACCR0 += ONE_BIT_TA_TICKS;		// Add Offset to CCR0.
      uart_ctxt.txval >>= 1;
      uart_ctxt.tx_bits_ctr--;
    } else {
       // If all bits TXed
      TACCTL0 &= ~CCIE ;		// Disable interrupt

      uart_ctxt.state &= ~UART_TXING;
      LED_SET(LED_TX, 0);
      __low_power_mode_off_on_exit();
      dbg.txdone++;
    }
  }
}



/*----------------------------------------------------------------------------*/
// Timer A CCR1 interrupt service routine
#if !defined(__GNUC__)
#pragma vector=TIMERA1_VECTOR
__interrupt
#else
__attribute__ ((interrupt(TIMER0_A1_VECTOR)))
#endif
void TimerA1_ISR (void)
{
  unsigned short compare_value;

  TACCTL1 &= ~CCIFG;
  if ((uart_ctxt.state & UART_RXING) != 0) {
    if (P1IN & RXD) {
      // If bit is set?
      uart_ctxt.rxval |= 0x0400;		// Set the value in the uart_ctxt.rxval
    }
    uart_ctxt.rxval >>= 1;		        // Shift the bits down
    uart_ctxt.rx_bits_ctr--;

    if (uart_ctxt.rx_bits_ctr > 0) {
      // Still bits left to receive.
      TACCR1 += ONE_BIT_TA_TICKS;
    } else {
      // Whole byte received.
      TACCTL1 |= (CCIE + CM_2 + CAP + SCS);   // Restore capture capability
      P1SEL |= RXD;                     // Restore peripheral mode on pin.

      if ( (uart_ctxt.rxval & 0x201) == 0x200) {
        // Validate the start and stop bits are correct

        uart_ctxt.rxval = uart_ctxt.rxval >> 1;	// Remove start bit
        uart_ctxt.rxval &= 0xFF;			// Remove stop bit

        __disable_interrupt();
#if USE_RX_RINGBUFFER
        ringbuf_put(&uart_ringbuffer, uart_ctxt.rxval);
#else
        if (  (UART_rx_buffer_idx < UART_RX_BUFFER_SIZE) ) {
          // Now char has been received, so we can stack in on
          UART_rx_buffer[UART_rx_buffer_idx++] = uart_ctxt.rxval;
        } else {
          //TODO: buffer full
        }
#endif
        __nop();
        __enable_interrupt();

      } else {
        //TODO: Hanlde RX error
        __nop();
      }

      dbg.rxdone++;

      uart_ctxt.state &= ~UART_RXING;
      LED_SET(LED_RX, 0);
      __low_power_mode_off_on_exit();
    }

  } else {
    dbg.rxstart++;
    // So, we have detected a start bit falling edge. We need to sample it again
    // in HALF_BIT_TA_TICKS, this time using compare register.

#if (HALF_BIT_TA_TICKS-ISR_DELAY_TICKS > START_BIT_TO_CCR_ASSIGN)
//#if 0
    compare_value = TACCR1 + (HALF_BIT_TA_TICKS - ISR_DELAY_TICKS);  // set next compare value.
    uart_ctxt.rx_bits_ctr = 10;		  // Load Bit counter, 8 bits + STOP
#else
    //In this case we have already passed the half-bit. So read directly
    //the start bit here.
    compare_value = TACCR1 + (ONE_BIT_TA_TICKS + HALF_BIT_TA_TICKS - ISR_DELAY_TICKS); // set next compare value.
    if ((P1IN & RXD) == 0) {
    uart_ctxt.rx_bits_ctr = 9;		  // Load Bit counter, 8 bits + STOP
    } else {
      //This is not a start bit or we are already in the 1st data bit => abort.
      return;
    }
#endif

    TACCTL1 &= ~(CM_3+CAP);         // Disable capture mode -> set compare mode.
    P1SEL &= ~RXD;                  // Disable peripheral mode on pin.

    TACCR1 = compare_value;         //We have to use compare value because, when
                                    //CCR module goes from Capture to Compare mode,
                                    //the TACCR1 value is set to TAR!!
    uart_ctxt.rxval = 0x00;		  // Initialize uart_ctxt.rxval

    uart_ctxt.state |= UART_RXING;
    LED_SET(LED_RX, 1);
  }
}


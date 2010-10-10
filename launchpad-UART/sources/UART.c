/******************************************************************************
 *                 Half Duplex Software UART on the LaunchPad
 * 
 * Description: UART Library.
 * Author: Marco Vedovati
 *
 ******************************************************************************/
/* Initial implementation from:
 * Author: Nicholas J. Conn - http://msp430launchpad.com
 * Email: webmaster at msp430launchpad.com
 ******************************************************************************/
  
#include <io430.h>
#include <in430.h>

#define		TXD             BIT1    // TXD on P1.1
#define		RXD		BIT2	// RXD on P1.2
#define         ACT_LED         P1OUT_bit.P1OUT_0

#define   BIT_TIME    	104		// 9600 Baud, SMCLK=1MHz (1MHz/9600)=104
#define	  BIT_TIME_5	52		// Time for half a bit.



unsigned char BitCnt;		// Bit count, used when transmitting byte
unsigned int TXByte;		// Value sent over UART when Transmit() is called
unsigned int RXByte;		// Value recieved once hasRecieved is set

unsigned char isReceiving;		// Status for when the device is receiving
unsigned char hasReceived;		// Lets the program know when a byte is received

// Function Definitions
void Transmit(void);

void initUART(void)
{
  P1SEL &= ~(TXD+BIT0);
  P1DIR |= (TXD+BIT0);
  
  P1IES |= RXD;				// RXD Hi/lo edge interrupt
  P1IFG &= ~RXD;			// Clear RXD (flag) before enabling interrupt
  P1IE |= RXD;				// Enable RXD interrupt
  
  
  isReceiving = 0;			// Set initial values
  hasReceived = 0;
}


void UART_echo_mode(void)
{
  if (hasReceived) {
    // If the device has recieved a value
    hasReceived = 0;	// Clear the flag
    TXByte = RXByte;	// Load the recieved byte into the byte to be transmitted
    Transmit();
  } else {
    __low_power_mode_0();        
    // LPM0, the ADC interrupt will wake the processor up. This is so that it does not
    //	endlessly loop when no value has been Received.
  }
}


void UART_tx_string (unsigned char * str)
{
  while ( *str != '\0' ) {
    TXByte = *str;
    Transmit();
    __low_power_mode_0(); // wait for TX completion.
    str++;
  }
}
// Function Transmits Character from TXByte 
void Transmit()
{ 
	while(isReceiving);			// Wait for RX completion
  	CCTL0 = OUT;				// TXD Idle as Mark
  	TACTL = TASSEL_2 + MC_2;		// SMCLK, continuous mode

  	BitCnt = 10;				// Load Bit counter, 8 bits + ST/SP
  	CCR0 = TAR;				// Initialize compare register
  
  	CCR0 += BIT_TIME;			// Set time till first bit
  	TXByte |= 0x100;			// Add stop bit to TXByte (which is logical 1)
  	TXByte = TXByte << 1;			// Add start bit (which is logical 0)
  
        CCTL0 = CCIE;
        ACT_LED ^=1;
  	//CCTL0 =  CCIS_0 + OUTMOD_0 + CCIE;	// Set signal, intial value, enable interrupts
  	//while ( CCTL0 & CCIE );		// Wait for previous TX completion
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{  	
  isReceiving = 1;
  
  P1IE &= ~RXD;			// Disable RXD interrupt
  P1IFG &= ~RXD;		// Clear RXD IFG (interrupt flag)
  
  TACTL = TASSEL_2 + MC_2;	// SMCLK, continuous mode
  CCR0 = TAR;			// Initialize compare register
  CCR0 += BIT_TIME_5;		// Set time till first bit
  CCTL0 = OUTMOD_1 + CCIE;	// Dissable TX and enable interrupts
  
  RXByte = 0;			// Initialize RXByte
  BitCnt = 0x9;			// Load Bit counter, 8 bits + ST
  
  
}

// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  if(!isReceiving)
  {
    CCR0 += BIT_TIME;			// Add Offset to CCR0  
    if ( BitCnt == 0)			// If all bits TXed
    {
      TACTL = TASSEL_2;		// SMCLK, timer off (for power consumption)
      CCTL0 &= ~ CCIE ;		// Disable interrupt
      __low_power_mode_off_on_exit();
      ACT_LED =0;
    }
    else
    {
      
      if (TXByte & 0x01) {
        //CCTL0 &= ~ OUTMOD_2;	   // If it should be 1, set it to 1
        P1OUT |= TXD;
      }
      else {
        //CCTL0 |=  OUTMOD_2;	   // Set TX bit to 0
        P1OUT &= ~TXD;
      }
      TXByte = TXByte >> 1;
      BitCnt --;
    }
  }
  else
  {
    CCR0 += BIT_TIME;				// Add Offset to CCR0  
    if ( BitCnt == 0)
    {
      TACTL = TASSEL_2;			// SMCLK, timer off (for power consumption)
      CCTL0 &= ~ CCIE ;			// Disable interrupt
      
      isReceiving = 0;
      
      P1IFG &= ~RXD;				// clear RXD IFG (interrupt flag)
      P1IE |= RXD;				// enabled RXD interrupt
      
      if ( (RXByte & 0x201) == 0x200)		// Validate the start and stop bits are correct
      {
        RXByte = RXByte >> 1;		// Remove start bit
        RXByte &= 0xFF;			// Remove stop bit
        hasReceived = 1;
      }
      __low_power_mode_off_on_exit();
    }
    else
    {
      if ( (P1IN & RXD) == RXD)		// If bit is set?
        RXByte |= 0x400;		// Set the value in the RXByte 
      RXByte = RXByte >> 1;		// Shift the bits down
      BitCnt --;
    }
  }
}

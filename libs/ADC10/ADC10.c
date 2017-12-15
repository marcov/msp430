#if defined(__GNUC__)
#include <msp430.h>
#else
#include <io430.h>
#include <in430.h>
#endif

#include "ADC10.h"
#include "soft_delay.h"

#define VREF_VALUE              (1.5f)
#define ADC10_MAX_DIGITAL_VALUE ((2<<10)-1)
#define MSP430_TEMPERATURE_CH   (10)

// Six values: we will remove the higher and the lower, then do the average
// of the remaining.
#define ADC10_TEMPERATURE_SAMPLES 6

/*----------------------------------------------------------------------------*/
void initADC10(void)
{
  //SREF = 1 : VR+ = VREF+ and VR- = VSS
  //SHT = 2 : Sample & hold time is x16 ADC10LK (so 128us with soure clock SMCLK/8)
  //REF2_5V = 0: Reference @ 1.5V
  //REFON = 1 : Reference generator is ON.
  //ADC10ON = 1: ADC10 is on.
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON;

  // SHS = 0 : sample&hold signal is ADC10SC bit
  //ADC10DF = 0 : binary format
  //SSEL = 3 : ADC10 clock source is SMCLK
  //ADC10DIV = 7 : clock divider /8
  // CONSEQ = 0: single channel, single conversion
  ADC10CTL1 = SHS_0 + CONSEQ_0 + ADC10DIV_7 + ADC10SSEL_3;

  __delay_ms(10); //Wait ADC ref to settle.
}


/*----------------------------------------------------------------------------*/
static void start_channel_acquisition(unsigned char channel)
{
  if (channel <= 0x0F){
    ADC10CTL1 &= INCH_15;
    ADC10CTL1 |= (channel<<12);
  }
  ADC10CTL0 |= ENC + ADC10SC;

}

/*----------------------------------------------------------------------------*/
static unsigned short get_ADC10_digital_value(void)
{
  while (ADC10CTL1 & ADC10BUSY);

  return ADC10MEM;
}


/*----------------------------------------------------------------------------*/
float read_ADC10_channel_float(unsigned char channel)
{
  start_channel_acquisition(channel);
  return ((float)get_ADC10_digital_value() / \
          (float)ADC10_MAX_DIGITAL_VALUE * (float)VREF_VALUE);
}


unsigned short read_ADC10_channel_digital(unsigned char channel)
{
  start_channel_acquisition(channel);
  return get_ADC10_digital_value();
}


/*----------------------------------------------------------------------------*/
unsigned short get_MSP430_temperature_digital(void)
{
  unsigned short temperature_avg = 0;
  // Setting min and max to the following values assures us that they are both
  // initialized to the first value read from ADC.
  unsigned short min = ADC10_MAX_DIGITAL_VALUE;
  unsigned short max = 0;
  unsigned short currsample;

  for (unsigned char sidx = 0; sidx < ADC10_TEMPERATURE_SAMPLES; sidx++) {
    start_channel_acquisition(MSP430_TEMPERATURE_CH);
    currsample = get_ADC10_digital_value();
    if (max < currsample)  max = currsample;
    if (min > currsample)  min = currsample;
    temperature_avg += currsample;
  }
  temperature_avg -= (max + min); //Remove highest and lower vals
  temperature_avg /= (ADC10_TEMPERATURE_SAMPLES - 2);

  return temperature_avg;
}

#ifndef __ADC10_H__
#define __ADC10_H__

void initADC10(void);
float read_ADC10_channel_float(unsigned char channel);
unsigned short read_ADC10_channel_digital(unsigned char channel);
unsigned short get_MSP430_temperature_digital(void);

#endif //__ADC10_H__
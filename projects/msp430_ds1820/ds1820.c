#if defined(__GNUC__)
#include <msp430.h>
#else
#include <io430.h>
#include <in430.h>
#endif
#include "ds1820.h"
#include "1wire.h"
#include "soft_delay.h"

#define START_T_CONVERSION	  0x44
#define WRITE_SCRATCHPAD	  0x4E
#define READ_SCRATCHPAD		  0xBE
#define SKIP_ROM_CHECK		  0xCC


/*
 * \brief Reads the scratchpad register of the DS1820 once the T conversion
 * has finished.
 * \param regbuffer pointer to a characte buffer of size SCRATCHPAD_REGISTER_SIZE
 * \return
 */
void DS1820_get_temperature_registers(unsigned char * regbuffer)
{
  unsigned char busy=0;

  onewire_reset();
  onewire_write(SKIP_ROM_CHECK);
  onewire_write(START_T_CONVERSION);

  __delay_ms(100);

  do {
    busy = onewire_read();
  } while (busy == 0);

  onewire_reset();
  onewire_write(SKIP_ROM_CHECK);
  onewire_write(READ_SCRATCHPAD);

  for (unsigned char i = 0 ; i<DS1820_SPAD_REGISTER_SIZE ; i++) {
          regbuffer[i] = onewire_read();
  }

}


/*
 * \brief Reads temperature from DS1820 and returns the value converted in
 * T*100 unit.
 * \param temperature pointer to memory location to store temperature.
 * \return
 */
void DS1820_get_temperature(signed short * temperature)
{

  unsigned char scratchpad_register[DS1820_SPAD_REGISTER_SIZE];

  // Get DS1820 temperature registers.
  DS1820_get_temperature_registers(scratchpad_register);


//1: Temperature with the standard resolution:
//Format:
//Abs(T) * 10.
//The MSB of temperature represent the sign!
/*
	*temperature = scratchpad_register[0];

	sign = scratchpad_register[1];

	// We want to store the absolute value.
 	if (sign > 0) {
 		*temperature = -(signed char)*temperature;
 	}

 	*temperature *=5;		// In this way we will get temperature x10 value.

	//Cowardly adding sign.
	if (sign > 0) {
 		*temperature |= 0x8000;
	}
*/

//2: Using float
// Format:
// float
/*
	float float_temp;
	float_temp = (scratchpad_register[0] | scratchpad_register[1]<<8)/2.0f - \
	           0.25f + \
	           ((float)(scratchpad_register[7] - scratchpad_register[6]) / (float)scratchpad_register[6]);
	*temperature = (signed short)float_temp;
*/

//3: Using integer with extended resolution.
// Format:
// T * 100 (signed short).
  *temperature = (scratchpad_register[0] | (scratchpad_register[1]<<8))*5*10 - \
                  25 + \
                 (100 * (scratchpad_register[7] - scratchpad_register[6]) / scratchpad_register[7]);
}


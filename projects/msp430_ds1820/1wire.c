// (C) copyright 2003 j.d.sandoz / jds-pic !at! losdos.dyndns.org

// released under the GNU GENERAL PUBLIC LICENSE (GPL)
// refer to http://www.gnu.org/licenses/gpl.txt

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

/***********************1Wire Class**********************
Description: This class handles all communication
between the processor and the 1wire
sensors.
********************************************************/
#if defined(__GNUC__)
#include <msp430.h>
#else
#include <io430.h>
#include <in430.h>
#endif

#include "1wire.h"
#include "board_support.h"
#include "soft_delay.h"


static inline void onewire_low(void)
{
    P1OUT |= (1 << ONEWIRE_BUS_PIN);
    P1DIR |= (1 << ONEWIRE_BUS_PIN);
}

static inline void onewire_hiz(void)
{
    P1DIR &= ~(1 << ONEWIRE_BUS_PIN);
}

static inline unsigned char onewire_get(void)
{
    P1DIR &= ~(1 << ONEWIRE_BUS_PIN);
    return (P1IN >> ONEWIRE_BUS_PIN) & 0x01;
}


/*******************1-wire communication functions********************/

/************onewire_reset*************************************************/
/*This function initiates the 1wire bus */
/* */
/*PARAMETERS: */
/*RETURNS: */
/*********************************************************************/

void onewire_reset()  // OK if just using a single permanently connected device
{
	onewire_low();
	__delay_us( 500uL ); // pull 1-wire low for reset pulse
	onewire_hiz(); // float 1-wire high
	__delay_us( 500uL ); // wait-out remaining initialisation window.
	onewire_hiz();
}

/*********************** onewire_write() ********************************/
/*This function writes a byte to the sensor.*/
/* */
/*Parameters: byte - the byte to be written to the 1-wire */
/*Returns: */
/*********************************************************************/

void onewire_write(unsigned char data)
{
	for (unsigned char count=0; count<8; ++count) {
		onewire_low();
		__delay_us( 2 ); // pull 1-wire low to initiate write time-slot.
		if ((data & 0x01) == 1) {
			onewire_hiz();
		} else {
			onewire_low();
		}
		data >>=1;
		__delay_us( 60 ); // wait until end of write slot.
		onewire_hiz(); // set 1-wire high again,
		__delay_us( 2 ); // for more than 1us minimum.
	}
}

/*********************** read1wire() *********************************/
/*This function reads the 8 -bit data via the 1-wire sensor. */
/* */
/*Parameters: */
/*Returns: 8-bit (1-byte) data from sensor */
/*********************************************************************/

unsigned char onewire_read()
{
	unsigned char data = 0x00;

	for (unsigned char count=0; count<8; count++)	{
		onewire_low();
		__delay_us( 2 ); // pull 1-wire low to initiate read time-slot.
		onewire_hiz(); // now let 1-wire float high,
		__delay_us( 8 ); // let device state stabilise,
		data |= (onewire_get() << count);

		//shift_right(&data,1,input(ONE_WIRE_PIN)); // and load result.
		__delay_us( 120 ); // wait until end of read slot.
	}

	return data;
}

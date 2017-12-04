#include <stdlib.h>
#include "shared.h"

/*
 * Converts a _2 digits_ integer to a representation in 2 characters.
 * The order of returned characters is big endian.
 * 
 * E.g. if i = 26 then the returned a = ['2', '6']
 * 
 * \param i: the integer number
 * \param a[]: a pointer to an array of 2 bytes.
 */
void u8_to_2digits_str (unsigned char i,  char * const a)
{
	div_t d = div(i , 10);
	
	a[0] = d.quot + 0x30;
	a[1] = d.rem + 0x30;
}

/*
 * Converts a _5 digits_ integer to a representation in 4 characters.
 * The order of returned characters is big endian.
 * 
 * E.g. if i = 1265 then the returned a = ['1', '2', '6', '5']
 * 
 * \param i: the integer number
 * \param a[]: a pointer to an array of 5 bytes.
 */
void u16_to_5digits_str (unsigned short i,  char * const a)
{
	div_t d = div(i , 10);
	a[4] = d.rem + 0x30;
	
	d = div(d.quot , 10);
	a[3] = d.rem + 0x30;

	d = div(d.quot , 10);
	a[2] = d.rem + 0x30;

	d = div(d.quot , 10);	
	a[0] = d.quot + 0x30;
	a[1] = d.rem + 0x30;
}

#ifndef __SOFT_DELAY_H
#define __SOFT_DELAY_H

#include "board_support.h"


//Without rounding:
#define __delay_us(x) __delay_cycles (MCLK_FREQUENCY / 1000000 * x)
#define __delay_ms(x) __delay_cycles (MCLK_FREQUENCY / 1000 * x)

/*
// With rounding:
#define __delay_us(x) __delay_cycles( ((MCLK_FREQUENCY*x*10/1000000) + 5 /10) )
#define __delay_ms(x) __delay_cycles( ((MCLK_FREQUENCY*x*10/1000) + 5 /10) )

*/

// With rounding (but compile warning):
//#define __delay_us(x) __delay_cycles( ((MCLK_FREQUENCY*x*10*1E-6) + 5 /10) )
//#define __delay_ms(x) __delay_cycles( ((MCLK_FREQUENCY*x*10*1E-3) + 5 /10) )



//#define __delay_us(x) __delay_cycles(x)
//#define __delay_ms(x) __delay_cycles(x*1000uL)



#endif //__SOFT_DELAY_H
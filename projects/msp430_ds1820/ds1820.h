#ifndef DS1820_H_
#define DS1820_H_

#define DS1820_SPAD_REGISTER_SIZE  9

void DS1820_get_temperature(signed short * temperature);
void DS1820_get_temperature_registers(unsigned char * regbuffer);

#endif /*DS1820_H_*/

#ifndef __1WIRE_H__
#define __1WIRE_H__

void onewire_reset();
void onewire_write(unsigned char data);
unsigned char onewire_read();

#endif /*1WIRE_H_*/

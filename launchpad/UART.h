#ifndef __UART_H__
#define __UART_H__


#define BAUDRATE                9600

void initUART(void);
void UART_echo_mode(void);
void UART_tx_string (char * str);
void start_UART_rx(void);
unsigned char UART_get_char(void);

#endif //__UART_H__
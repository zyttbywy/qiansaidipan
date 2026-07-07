#ifndef _UART_H_
#define _UART_H_

#include "zf_common_fifo.h"

extern int use_data[9];
extern int x_speed;
extern int y_speed;
extern int o_speed;

extern fifo_struct uart2_data_fifo;

void my_uart_write_byte(unsigned char byte);
void Data_Send(int* pst);
void get_expect_speed(void);
void get_expect_speed_receiver(void);
#endif // _UART_H_
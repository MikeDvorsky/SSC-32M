/*
 * uart.h
 *
 * Serial input/output for the DeskPet servo controller.
 * Author : Mike Dvorsky
 */ 


#ifndef UART_H
#define UART_H

void uart_init(void);
void uart_update(void);
bool uart_rx_get_char(uint8_t * rxByte);
void uart_tx_put_char(uint8_t txByte);
void uart_tx_string(uint8_t * s);
void uart_tx_uint16(uint16_t num);

#if (UNIT_TEST)
void uart_rx_stuff(char * cmd_string);
#endif

#endif //UART_H

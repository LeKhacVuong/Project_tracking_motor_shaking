/*
 * board.h
 *
 *  Created on: Dec 26, 2023
 *      Author: vypa0
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_


#include "main.h"

#define RS485_PORT_COM			USART1
#define UART_BAUDRATE_SLAVE		115200


typedef struct UART_hw_t UART_hw;

struct UART_hw_t{
	UART_HandleTypeDef 	uart_module;
	DMA_HandleTypeDef	uart_DMA;
	uint8_t				rx_data;

};

extern UART_hw rs485_com;


void board_init(void);
void lkv_lcd_float_to_string(double* value, uint8_t* buffer);


#endif /* BOARD_BOARD_H_ */

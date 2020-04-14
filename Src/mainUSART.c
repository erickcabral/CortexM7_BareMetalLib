/*
 * mainUSART.c
 *
 *  Created on: 14 Apr 2020
 *      Author: Erick Cabral
 */
#include "usartDriver.h"

int main(void) { //USART TEST
	RCC_Reg_t rcc;
	initializeRCC(&rcc);

	USART_Reg_t usart1;
	enableUSARTx(&rcc, USART1);
	usartX_initialize(&usart1, USART1_BASEADDR);
	usart_setCommMode(&usart1, RX_TX);
	usart_setParity(&usart1, ENABLE, PARITY_EVEN);
	usart_setWordLength(&usart1, LENGHT_7_BITS);

	while (TRUE) {

	}
}

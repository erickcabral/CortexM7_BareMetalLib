/*
 * usartDriver.c
 *
 *  Created on: 13 Apr 2020
 *      Author: Erick Cabral
 */
#include "usartDriver.h"

void enableUSARTx(RCC_Reg_t *pRCC, uint8_t usartX) {
	/* SETUP POINTERS */
	if (pRCC->pAPB2ENR != (__volU32*) APB2ENR
			|| pRCC->pC1_APB2ENR != (__volU32*) C1_APB2ENR) {
		pRCC->pAPB2ENR = (__volU32*) APB2ENR;
		//pRCC->pC1_APB2ENR = (__volU32*) C1_APB2ENR;
	} else if ((pRCC->pAPB1LENR != (__volU32*) APB1LENR)
			|| (pRCC->pC1_APB1LENR != (__volU32*) C1_APB1LENR)) {
		pRCC->pAPB1LENR = (__volU32*) APB1LENR;
		//pRCC->pC1_APB1LENR = (__volU32*) C1_APB1LENR;
	}
	/*SELECT CORRECT USART*/
	switch (usartX) {
	case 1:
	case 6:
		if (usartX == 1) {
			*(pRCC->pAPB2ENR) |= (1 << 4);
			*(pRCC->pAPB2ENR) |= (1 << 4);
		} else {
			*(pRCC->pAPB2ENR) |= (1 << 5);
			*(pRCC->pAPB2ENR) |= (1 << 5);
		}
		break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 7:
		*(pRCC->pAPB1LENR) |= (1 << usartX);
		break;
	}
}
/**
 *  INITIALIZING DEFAULT CONFIG
 *   TRANSMISSION 	-> RX_TX
 *   WORD LENGTH 	->
 *   STOP BITS 		->
 *   PARITY 		->
 *   BAUND RATE 	->
 *   FLOW CONTROL 	->
 */
void usartX_initialize(USART_Reg_t *pUSARTx, uint32_t usartX_baseaddr) {
	uint32_t USART_BASEADDR;
	switch (usartX_baseaddr) {
	case USART1_BASEADDR:
		USART_BASEADDR = USART1_BASEADDR;
		break;
	case USART2_BASEADDR:
		USART_BASEADDR = USART2_BASEADDR;
		break;
	case USART3_BASEADDR:
		USART_BASEADDR = USART3_BASEADDR;
		break;
	case UART4_BASEADDR:
		USART_BASEADDR = UART4_BASEADDR;
		break;
	case UART5_BASEADDR:
		USART_BASEADDR = UART5_BASEADDR;
		break;
	case USART6_BASEADDR:
		USART_BASEADDR = USART6_BASEADDR;
		break;
	case UART7_BASEADDR:
		USART_BASEADDR = UART7_BASEADDR;
		break;
	case UART8_BASEADDR:
		USART_BASEADDR = UART8_BASEADDR;
		break;
	}

	pUSARTx->pCR1 = (__volU32*) (USART_BASEADDR + 0x00U);
	pUSARTx->pCR2 = (__volU32*) (USART_BASEADDR + 0x04U);
	pUSARTx->pCR3 = (__volU32*) (USART_BASEADDR + 0x08U);
	pUSARTx->pBRR = (__volU32*) (USART_BASEADDR + 0x0CU);
	//..
	pUSARTx->pRDR = (__volU32*) (USART_BASEADDR + 0x24U);
	pUSARTx->pTDR = (__volU32*) (USART_BASEADDR + 0x28U);

	/* WORD_LENGTH -> 8BITS */
	*(pUSARTx->pCR1) |= (0 << 28); 	// (M1) WORD LENGHT;
	*(pUSARTx->pCR1) |= (0 << 12); 	// (M0) WORD LENGHT;

	/* COMM_TYPE -> RX TX */
	*(pUSARTx->pCR1) |= (1 << 3);				// (TX) USART ENABLE
	*(pUSARTx->pCR1) |= (1 << 2);  				// (RX) ENABLE;

	/*(OVER8) OVERSAMPLING*/
	*(pUSARTx->pCR1) |= (1 << 15); 				// (OVER8) OVERSAMPLING 8 or 16;

	/* PARITY -> ENABLE */
	*(pUSARTx->pCR1) |= (1 << 10); 		// (PCE) PARITY ENABLE;

	//uint8_t STOP_BITS = STOP_1_0;
	//uint8_t BAUND_RATE = BAUD_9600;
	//uint8_t FLOW_CONTROL = CTS;
	//TODO

	/*USART ENABLE */
	*(pUSARTx->pCR1) |= (1 << 0); 				// (UE) USART ENABLE;
}

/* SETUP USART/UART COMMUNICATION MODE
 * CR1 RX BIT -> 2
 * CR1 TX BIT -> 3
 */
void usart_setCommMode(USART_Reg_t *pUSARTx, uint8_t commMode) {
	switch (commMode) {
	case RX_ONLY:
		setOneBitRegister(pUSARTx->pCR1, 2, ENABLE); //(RX) ENABLE (CR1 RX BIT -> 2);
		setOneBitRegister(pUSARTx->pCR1, 3, DISABLE); //(TX) ENABLE (CR1 RX BIT -> 3);
		break;
	case TX_ONLY:
		setOneBitRegister(pUSARTx->pCR1, 2, DISABLE); //(RX) ENABLE (CR1 RX BIT -> 2);
		setOneBitRegister(pUSARTx->pCR1, 3, ENABLE); //(TX) ENABLE (CR1 RX BIT -> 3);
		break;
	case RX_TX:
		setOneBitRegister(pUSARTx->pCR1, 2, ENABLE); //(RX) ENABLE (CR1 RX BIT -> 2);
		setOneBitRegister(pUSARTx->pCR1, 3, ENABLE); //(TX) ENABLE (CR1 RX BIT -> 3);
		break;
	}
}
/* SETUP USART/UART PARITY MODE
 * PARITY -> ENABLED/DISABLED
 * PARIT MODE -> PARITY_ODD / PARITY_EVEN
 */
void usart_setParity(USART_Reg_t *pUSARTx, boolean enable, uint8_t parity) {
	setOneBitRegister(pUSARTx->pCR1, 10, enable); // (PCE) PARITY ENABLE ( CR1 BIT -> 10;
	if (enable) {
		//TODO: SET ODD or EVEN ....
	}
}
/* SETUP USART/UART WORD LENGTH
 *	BITS_8 			0	//00: 1 start bit, 8 Data bits, n Stop bit
 *	BITS_9 			1 	//01: 1 start bit, 9 Data bits, n Stop bit
 *	BITS_7 			2	//10: 1 start bit, 7 Data bits, n Stop bit
 */
void usart_setWordLength(USART_Reg_t *pUSARTx, uint8_t word_length) {
	usart_disable(pUSARTx, DISABLE);
	switch (word_length) {
	case LENGTH_8_BITS: //
		setOneBitRegister(pUSARTx->pCR1, 28, 0); // (M1) WORD LENGHT;
		setOneBitRegister(pUSARTx->pCR1, 12, 0); // (M0) WORD LENGHT;
		break;
	case LENGTH_9_BITS:
		setOneBitRegister(pUSARTx->pCR1, 28, 0); // (M1) WORD LENGHT;
		setOneBitRegister(pUSARTx->pCR1, 12, 1); // (M0) WORD LENGHT;
		break;
	case LENGHT_7_BITS:
		setOneBitRegister(pUSARTx->pCR1, 28, 1); // (M1) WORD LENGHT;
		setOneBitRegister(pUSARTx->pCR1, 12, 1); // (M0) WORD LENGHT;
		break;
	}
	usart_disable(pUSARTx, ENABLE);
}

void usart_disable(USART_Reg_t *pUSARTx, boolean enable) {
	setOneBitRegister(pUSARTx->pCR1, 0, enable);
}

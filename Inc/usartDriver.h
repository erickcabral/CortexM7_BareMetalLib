/*
 * usartDriver.h
 *
 *  Created on: 13 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef USARTDRIVER_H_
#define USARTDRIVER_H_

#include"stm32H743xx.h"

#define USART1				1	//APB2(D2)
#define USART2				2 	//APB1(D2)
#define USART3				3	//APB1(D2)
#define UART4				4	//APB1(D2)
#define UART5				5	//APB1(D2)
#define USART6				6	//APB2(D2)
#define UART7				7	//APB1(D2)
#define UART8				8	//APB1(D2)

#define USART1_BASEADDR		0x40011000	//APB2(D2)
#define USART6_BASEADDR		0x40011400	//APB2(D2)

#define USART2_BASEADDR		0x40004400 	//APB1(D2)
#define USART3_BASEADDR		0x40004800	//APB1(D2)
#define UART4_BASEADDR		0x40004C00	//APB1(D2)
#define UART5_BASEADDR		0x40005000	//APB1(D2)
#define UART7_BASEADDR		0x40007800	//APB1(D2)
#define UART8_BASEADDR		0x40007C00	//APB1(D2)

typedef struct {
	__volU32 *pCR1; // OFFSET 0x00U;
	__volU32 *pCR2; // OFFSET 0x04U;
	__volU32 *pCR3; // OFFSET 0x08U;
	__volU32 *pBRR; // OFFSET 0x0CU;
	//..
	__volU32 *pRDR; // OFFSET 0x24U;
	__volU32 *pTDR; // OFFSET 0x28U;
} USART_Reg_t;

// Transmission/Reception
// Bit Length
// Stop Bits
// Parity
// Baund Rate
// Flow Control

/**
 * USART CONFIG OPTIONS *
 */

/*<! COMMUNICATION TYPE >*/
#define RX_ONLY		1
#define TX_ONLY		2
#define RX_TX		3

/*<! FLOW CONTROL TYPE >*/
#define CTS		1 //Clear to Send
#define RTS		2 //Request to Send

/*<! BAUD RATE >*/
#define BAUD_1200		1200
#define BAUD_2400		2400
#define BAUD_4800		4800
#define BAUD_9600		9600
#define BAUD_19200		19200
#define BAUD_38400		38400
#define BAUD_57600		57600

/*<! WORD LENGHT >*/
#define LENGTH_8_BITS 			0	//00: 1 start bit, 8 Data bits, n Stop bit
#define LENGTH_9_BITS 			1 	//01: 1 start bit, 9 Data bits, n Stop bit
#define LENGHT_7_BITS 			2	//10: 1 start bit, 7 Data bits, n Stop bit

/*<! PARITY -> ENABLE or DISABLE >*/
#define PARITY_EVEN 	0 	//01: 1 start bit, 9 Data bits, n Stop bit
#define PARITY_ODD 		1	//00: 1 start bit, 8 Data bits, n Stop bit

/*<! STOP BITS >*/
#define STOP_1_0		0	//0b00 1 BIT
#define STOP_0_5		1	//0b01 0.5 BIT
#define STOP_2_0		2	//0b10 2 BIT
#define STOP_1_5		3	//0b11 1.5 BIT


void enableUSARTx(RCC_Reg_t *pRCC, uint8_t usartX);
void usart_disable(USART_Reg_t *pUSARTx, uint8_t enable);
void usartX_initialize(USART_Reg_t *pUSARTx, uint32_t usartX_baseaddr);
void usart_setCommMode(USART_Reg_t *pUSARTx, uint8_t commMode);
void usart_setParity(USART_Reg_t *pUSARTx, boolean enable, uint8_t parity);
void usart_setWordLength(USART_Reg_t *pUSARTx, uint8_t word_length);

#endif /* USARTDRIVER_H_ */

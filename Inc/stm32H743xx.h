/*
 * stm32H743xx.h
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef STM32H743XX_H_
#define STM32H743XX_H_

#include <stdint-gcc.h>

/*<! COMMOM VARIABLES >*/
#define __uint8				uint8_t
#define __int32U			uint32_t

#define __volU16			volatile uint16_t
#define __volU32			volatile uint32_t

#define boolean				__uint8

#define TRUE				1
#define FALSE				0
#define ENABLE				TRUE
#define DISABLE				FALSE

#define HIGH				TRUE
#define LOW					FALSE

#define RCC_BASEADDR		0x58024400U

#define APB1LENR 			(RCC_BASEADDR + 0x0E8U)
#define C1_APB1LENR 		(RCC_BASEADDR + 0x148U)

#define APB2ENR				(RCC_BASEADDR + 0x0F0U)
#define C1_APB2ENR			(RCC_BASEADDR + 0x150U)

/*<! RCC CONFIG API >*/
typedef struct {
	__volU32 *pAHB4ENR; // = (__volU32*)(RCC_BASEADDR + 0x0E0U);
	__volU32 *pC1_AHB4ENR; // = (__volU32*)(RCC_BASEADDR + 0x140U);

	__volU32 *pAPB4ENR; // = (__volU32*)(RCC_BASEADDR + 0x0F4U);
	__volU32 *pC1_APB4ENR; // = (__volU32*)(RCC_BASEADDR + 0x154U);

	__volU32 *pAPB1LENR; // = (__volU32*)(RCC_BASEADDR + 0x0E8U);
	__volU32 *pC1_APB1LENR; // = (__volU32*)(RCC_BASEADDR + 0x148U);

	__volU32 *pAPB2ENR; 		// OFFSET 0x0F0U;
	__volU32 *pC1_APB2ENR; 		// OFFSET 0x150;

} RCC_Reg_t;

void initializeRCC(RCC_Reg_t *pRCC);
void enableSYSCFG(RCC_Reg_t *pRCC, boolean enable);
void enableGPIOx(RCC_Reg_t *pRCC, uint8_t gpioX, boolean enable);

/*<! SUPPORT FUCNTIONS >*/
void setOneBitRegister(__volU32 *pRegister, uint8_t leastBit, uint8_t newValue);
void setTwoBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue);

#endif /* STM32H743XX_H_ */

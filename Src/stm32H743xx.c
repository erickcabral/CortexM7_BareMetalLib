/*
 * stm32H743xx.c
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#include "stm32H743xx.h"

void initializeRCC(RCC_Reg_t *pRCC) {
	pRCC->pAHB4ENR = ((__volU32*) (RCC_BASEADDR + 0x0E0U));
	pRCC->pC1_AHB4ENR = (__volU32*) (RCC_BASEADDR + 0x140U);

	pRCC->pAPB4ENR = (__volU32*) (RCC_BASEADDR + 0x0F4U);
	pRCC->pC1_APB4ENR = (__volU32*) (RCC_BASEADDR + 0x154U);

	pRCC->pAPB1LENR = (__volU32*) (RCC_BASEADDR + 0x0E8U);
	pRCC->pC1_APB1LENR = (__volU32*) (RCC_BASEADDR + 0x148U);

	pRCC->pAPB2ENR = (__volU32*) (RCC_BASEADDR + 0x0F0U);
	pRCC->pC1_APB2ENR = (__volU32*) (RCC_BASEADDR + 0x150);
}

void enableGPIOx(RCC_Reg_t *pRCC, uint8_t gpioX, boolean enable) {
	if (enable) {
		*(pRCC->pAHB4ENR) |= (1 << gpioX);
		//*(RCC->pC1_AHB4ENR) |= (1 << gpioX);
	} else {
		*(pRCC->pAHB4ENR) &= ~(1 << gpioX);
		//*(RCC->pC1_AHB4ENR) |= (1 << gpioX);
	}
}

void enableSYSCFG(RCC_Reg_t *pRCC, boolean enable) {
	*(pRCC->pAPB4ENR) |= (1 << 1); //SYSCFGEN = 1;
}

/*<!   >*/
void setOneBitRegister(__volU32 *pRegister, uint8_t leastBit, uint8_t newValue) {
	*pRegister &= ~((1 << leastBit));
	*pRegister |= (newValue << leastBit);
}

void setTwoBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue) {
	*pRegister &= ~((1 << (leastBit + 1)) | (1 << leastBit));
	*pRegister |= (newValue << leastBit);
}


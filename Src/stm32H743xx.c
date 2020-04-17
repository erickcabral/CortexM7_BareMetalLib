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
	pRCC->pC1_APB2ENR = (__volU32*) (RCC_BASEADDR + 0x150U);

	pRCC->pD3CCIPR = (__volU32*) (RCC_BASEADDR + 0x058U);
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
void resetBit(__volU32 *pRegister, uint8_t leastBit, uint8_t numberOfBits) {
	switch (numberOfBits) {
	case 1:
		*pRegister &= ~((1 << leastBit));
		break;
	case 2:
		*pRegister &= ~((1 << (leastBit + 1)) | (1 << leastBit));
		break;
	case 3:
		*pRegister &= ~((1 << (leastBit + 2)) | (1 << (leastBit + 1))
				| (1 << leastBit));
		break;
	case 4:
		*pRegister &= ~((1 << (leastBit + 3)) | (1 << (leastBit + 2))
				| (1 << (leastBit + 1)) | (1 << leastBit));
		break;
	}
}
void setOneBitRegister(__volU32 *pRegister, uint8_t leastBit, boolean enable) {
	if (enable) {
		*pRegister |= (enable << leastBit);
	} else {
		resetBit(pRegister, leastBit, 1);
	}
}

void setTwoBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue) {
	resetBit(pRegister, leastBit, 2);
	*pRegister |= (newValue << leastBit);
}
void setThreeBitRegister(__volU32 *pRegister, uint8_t leastBit,
		uint16_t newValue) {
	resetBit(pRegister, leastBit, 3);
	*pRegister |= (newValue << leastBit);
}
void setForBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue) {
	resetBit(pRegister, leastBit, 4);
	*pRegister |= (newValue << leastBit);
}

uint8_t getRegisterValue(__volU32 *pRegister, uint8_t bitToRead) {
	return (*(pRegister) >> bitToRead);
}

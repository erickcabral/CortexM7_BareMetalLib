/*
 * stm32H743xx.c
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#include "stm32H743xx.h"

void rcc_initialize(RCC_Reg_t *pRCC) {

	pRCC->pCR = ((__volU32*) (RCC_BASEADDR + 0x00U));

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
void rcc_setHSIClock(RCC_Reg_t *pRCC, uint8_t sys_clk_xMhz) {
	uint8_t selectedClock = 0;
	switch (sys_clk_xMhz) {
	case SYS_CLK_64_MHZ:
		selectedClock = 0;
		break;
	case SYS_CLK_32_MHZ:
		selectedClock = 1;
		break;
	case SYS_CLK_16_MHZ:
		selectedClock = 2;
		break;
	case SYS_CLK_8_MHZ:
		selectedClock = 3;
		break;
	}
	*(pRCC->pCR) |= (selectedClock << 3); //REG BIT HSIDIV
}
void rcc_enableGPIOx(RCC_Reg_t *pRCC, uint8_t gpioX, boolean enable) {
	if (enable) {
		*(pRCC->pAHB4ENR) |= (1 << gpioX);
		//*(RCC->pC1_AHB4ENR) |= (1 << gpioX);
	} else {
		*(pRCC->pAHB4ENR) &= ~(1 << gpioX);
		//*(RCC->pC1_AHB4ENR) |= (1 << gpioX);
	}
}

void rcc_enableSYSCFG(RCC_Reg_t *pRCC, boolean enable) {
	*(pRCC->pAPB4ENR) |= (1 << 1); //SYSCFGEN = 1;
}

/*********************************** SYSTICK API *****************************************/
/********************* SYSTICK FUNCTIONS ********************************/
void sysTick_setup(SysTick_Reg_t *pSysTick,
boolean clock_source, boolean genException, uint8_t sysClockMhz) {

	pSysTick->systemClockMhz = sysClockMhz;
	pSysTick->pCSR = (__volU32*) 0xE000E010;
	pSysTick->pRVR = (__volU32*) 0xE000E014;
	pSysTick->pCVR = (__volU32*) 0xE000E018;
	pSysTick->pCALIB = (uint32_t*) 0xE000E01C;

	resetRegister(pSysTick->pCSR, SYST_CSR_ENABLE, 1);
	setRegister(pSysTick->pCSR, clock_source, SYST_CSR_CLKSOURCE, 1);
	setRegister(pSysTick->pCSR, genException, SYST_CSR_TICKINT, 1);

}
void sysTick_startSysTick(SysTick_Reg_t *pSysTick, uint16_t timerMillis) {
	resetRegister(pSysTick->pCSR, SYST_CSR_ENABLE, 1);
	*(pSysTick->pCVR) = 0; //RESET CURRENT VAL
	*(pSysTick->pRVR) = 0; //RESET RELOAD

	uint32_t reload = (timerMillis * (pSysTick->systemClockMhz * 1000)) - 1; //1ms Clock (10e3) for 1us(10e6)
	*(pSysTick->pRVR) |= (reload << SYST_RVR_RELOAD); //RELOAD TIME
	setRegister(pSysTick->pCSR, ENABLE, SYST_CSR_ENABLE, 1);
}
void sysTick_stopSysTick(SysTick_Reg_t *pSysTick) {
	resetRegister(pSysTick->pCSR, SYST_CSR_ENABLE, 1);
}

void sysTick_delayMs(SysTick_Reg_t *pSysTick, uint16_t milliseconds) {

	if (milliseconds > 1000) { //Running more then 1 second.
		sysTick_startSysTick(pSysTick, 1000);
		for (int i = 0; i <= (milliseconds / 1000); i++) {
			while (sysTick_getState(pSysTick) != 1) {

			}
		}
	} else { //RUNNING 1 second Max time
		sysTick_startSysTick(pSysTick, milliseconds);
		while (sysTick_getState(pSysTick) != 1) {

		}
	}
}

boolean sysTick_getState(SysTick_Reg_t *pSysTick) {
	return (*(pSysTick->pCSR) >> SYST_CSR_COUNTFLAG);
}

/************************************* SUPPORT FUNCTIONS *****************************************/
void resetRegister(__volU32 *pRegister, uint8_t leastBit, uint8_t numberOfBits) {

	for (uint8_t i = 0; i < numberOfBits; i++) {
		*pRegister &= ~(1 << (leastBit + i));
	}
	/*
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
	 */
}
void setOneBitRegister(__volU32 *pRegister, uint8_t leastBit, boolean enable) {
	if (enable) {
		*pRegister |= (enable << leastBit);
	} else {
		resetRegister(pRegister, leastBit, 1);
	}
}
void setRegister(__volU32 *pRegister, uint32_t value, uint8_t leastBit,
		uint8_t numOfBits) {
	resetRegister(pRegister, leastBit, numOfBits);
	*pRegister |= (value << leastBit);
}

void setTwoBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue) {
	resetRegister(pRegister, leastBit, 2);
	*pRegister |= (newValue << leastBit);
}
void setThreeBitRegister(__volU32 *pRegister, uint8_t leastBit,
		uint16_t newValue) {
	resetRegister(pRegister, leastBit, 3);
	*pRegister |= (newValue << leastBit);
}
void setForBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue) {
	resetRegister(pRegister, leastBit, 4);
	*pRegister |= (newValue << leastBit);
}

boolean getRegisterValue(__volU32 *pRegister, uint8_t bitToRead) {
	boolean value = (*(pRegister) >> bitToRead);
	return value;
}

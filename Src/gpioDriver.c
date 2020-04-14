/*
 * gpioDriver.c
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#include "gpioDriver.h"

void initializeGPIOx(GPIOx_Confg_t* pGPIOx, uint8_t gpioX) {
	uint32_t gpio_baseAddr = gpio_GetGPIOxBASEADDRESS(gpioX);
	pGPIOx->pMODER = (__volU32*) (gpio_baseAddr + 0x00U);
	pGPIOx->pOTYPER = (__volU32*) (gpio_baseAddr + 0x04U);
	pGPIOx->pOSPEEDR = (__volU32*) (gpio_baseAddr + 0x08U);
	pGPIOx->pPUPDR = (__volU32*) (gpio_baseAddr + 0x0CU);
	pGPIOx->pIDR = (__volU32*) (gpio_baseAddr + 0x10U);
	pGPIOx->pODR = (__volU32*) (gpio_baseAddr + 0x14U);
	pGPIOx->pBSRR = (__volU32*) (gpio_baseAddr + 0x18U);
	pGPIOx->pLCKR = (__volU32*) (gpio_baseAddr + 0x1CU);
	pGPIOx->pAFRL = (__volU32*) (gpio_baseAddr + 0x20U);
	pGPIOx->pAFRH = (__volU32*) (gpio_baseAddr + 0x24U);
}

void setGPIOxModer(GPIOx_Confg_t *pGPIOx, uint16_t pinNumber, uint8_t pinMode) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(pGPIOx->pMODER, bitShifter, pinMode);
}
void setGPIOxType(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint8_t pinType) {
	setOneBitRegister(gpioX->pOTYPER, pinNumber, pinType);
}

void setGPIOxSpeed(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint8_t pinSpeed) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(gpioX->pOSPEEDR, bitShifter, pinSpeed);
}
void setGPIOxResistor(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint8_t pinPUPD) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(gpioX->pPUPDR, bitShifter, pinPUPD);
}

boolean getGPIOxState(GPIOx_Confg_t *gpioX, uint16_t pinNumber) {
	return (boolean) (*(gpioX->pODR) >> pinNumber);
}

void setGPIOxOutput(GPIOx_Confg_t *pGPIOx, uint16_t pinNumber, boolean high_low) {
	setOneBitRegister(pGPIOx->pODR, pinNumber, high_low);
}

void setGPIOxBSRR(GPIOx_Confg_t *gpioX, uint16_t pinNumber, boolean set_reset) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(gpioX->pBSRR, bitShifter, set_reset);
}
void setGPIOxLOCK(GPIOx_Confg_t *gpioX, uint16_t pinNumber, boolean locked);

void setGPIOxALTFunc(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint16_t altFunc);

/**
 *  @ SUPPORT FUNCTIONS
 */
uint32_t gpio_GetGPIOxBASEADDRESS(uint8_t gpioX) {
	switch (gpioX) {
	case GPIOA:
		return GPIOA_BASEADDR;
	case GPIOB:
		return GPIOB_BASEADDR;
	case GPIOC:
		return GPIOC_BASEADDR;
	}
	return 0x0;
}

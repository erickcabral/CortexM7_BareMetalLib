/*
 * gpioDriver.c
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#include "gpioDriver.h"

void gpio_initializeGPIOx(GPIOx_Confg_t *pGPIOx, uint8_t gpioX) {
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

void setGPIOxBSRR(GPIOx_Confg_t *gpioX, uint16_t pinNumber, boolean state) {
	if (state == HIGH) {
		*(gpioX->pBSRR) |= (1 << pinNumber); // SET HIGH (Bits [15:0])
	} else {
		*(gpioX->pBSRR) |= (1 << (pinNumber + 16)); //SET LOW (Bits [31:16])
	}
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

/*<! SET DEFAUT OUTPUT >*/
void gpio_defaultPinOutput(GPIOx_Confg_t *pGPIOx, uint8_t pinNumber) {
	setRegister(pGPIOx->pMODER, MODE_OUTPUT, (pinNumber * 2), 2);
	setRegister(pGPIOx->pOTYPER, TYPE_PUSH_PULL, (pinNumber), 1);
	setRegister(pGPIOx->pOSPEEDR, SPEED_LOW, (pinNumber * 2), 2);
	setRegister(pGPIOx->pPUPDR, PUPD_UP, (pinNumber * 2), 2);
}

void gpio_setPinState(GPIOx_Confg_t *pGPIOx, uint8_t pinNumber, boolean state) {
	setGPIOxBSRR(pGPIOx, pinNumber, state);
}

boolean gpio_getPinState(GPIOx_Confg_t *pGPIOx, uint8_t pinNumber) {
	boolean value = (*(pGPIOx->pODR) >> pinNumber);
	return value;
}

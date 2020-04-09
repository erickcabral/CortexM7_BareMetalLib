/*
 * gpioDriver.c
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#include "gpioDriver.h"

void initializeGPIOx(GPIOx_Confg_t *gpioX, uint32_t gpio_baseAddr) {
	gpioX->pMODER = (__volU32*) (gpio_baseAddr + 0x00U);
	gpioX->pOTYPER = (__volU32*) (gpio_baseAddr + 0x04U);
	gpioX->pOSPEEDR = (__volU32*) (gpio_baseAddr + 0x08U);
	gpioX->pPUPDR = (__volU32*) (gpio_baseAddr + 0x0CU);
	gpioX->pIDR = (__volU32*) (gpio_baseAddr + 0x10U);
	gpioX->pODR = (__volU32*) (gpio_baseAddr + 0x14U);
	gpioX->pBSRR = (__volU32*) (gpio_baseAddr + 0x18U);
	gpioX->pLCKR = (__volU32*) (gpio_baseAddr + 0x1CU);
	gpioX->pAFRL = (__volU32*) (gpio_baseAddr + 0x20U);
	gpioX->pAFRH = (__volU32*) (gpio_baseAddr + 0x24U);
}

void setGPIOxModer(GPIOx_Confg_t *pGPIOx, uint16_t pinNumber, uint8_t pinMode) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(pGPIOx->pMODER, bitShifter, pinMode);
}
void setGPIOxType(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint8_t pinType){
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
	return (boolean)(*(gpioX->pODR) >> pinNumber);
}

void setGPIOxOutput(GPIOx_Confg_t *pGPIOx, uint16_t pinNumber, boolean high_low){
	setOneBitRegister(pGPIOx->pODR, pinNumber, high_low);
}

void setGPIOxBSRR(GPIOx_Confg_t *gpioX, uint16_t pinNumber, boolean set_reset) {
	uint8_t bitShifter = (pinNumber * 2);
	setTwoBitRegister(gpioX->pBSRR, bitShifter, set_reset);
}
void setGPIOxLOCK(GPIOx_Confg_t *gpioX, uint16_t pinNumber, boolean locked);

void setGPIOxALTFunc(GPIOx_Confg_t *gpioX, uint16_t pinNumber, uint16_t altFunc);

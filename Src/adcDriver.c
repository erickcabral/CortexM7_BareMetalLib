/*
 * adcDriver.c
 *
 *  Created on: 15 Apr 2020
 *      Author: Erick Cabral
 */

#include "adcDriver.h"

void adc_enable(RCC_Reg_t *pRCC, uint8_t adcX) {
	switch (adcX) {
	case ADC1:
	case ADC2:
		if (adcX == ADC1) {
			setOneBitRegister(pRCC->pAHB1ENR, RCC_AHB4ENR_ADC12EN, 1);
		} else {
			setOneBitRegister(pRCC->pAHB1ENR, RCC_AHB4ENR_ADC12EN, 1);
		}
		break;
	case ADC3:
		setOneBitRegister(pRCC->pAHB4ENR, RCC_AHB4ENR_ADC3EN, 1);
		break;
	}
}
void adc_initialize(ADC_Handler_t *pADC_Handler, uint8_t gpioX, uint8_t adcX) {
	/*< GPIOx REGISTER INITIALIZATION >*/
	initializeGPIOx(&pADC_Handler->gpioX, gpioX);

	/*< ADCx REGISTER INITIALIZATION >*/
	uint32_t adc_baseAddress = adc_getADC_baseaddress(adcX);
	pADC_Handler->adcReg.pISR = (__volU32*) (adc_baseAddress + 0x00U);
	pADC_Handler->adcReg.pIER = (__volU32*) (adc_baseAddress + 0x04U);
	pADC_Handler->adcReg.pCR = (__volU32*) (adc_baseAddress + 0x08U);
	pADC_Handler->adcReg.pCFGR = (__volU32*) (adc_baseAddress + 0x0CU);
	pADC_Handler->adcReg.pCFGR2 = (__volU32*) (adc_baseAddress + 0x10U);
	pADC_Handler->adcReg.pSMPR1 = (__volU32*) (adc_baseAddress + 0x14U);
	pADC_Handler->adcReg.pSMPR2 = (__volU32*) (adc_baseAddress + 0x18U);
	pADC_Handler->adcReg.pPCSEL = (__volU32*) (adc_baseAddress + 0x1CU);
	pADC_Handler->adcReg.pSQR1 = (__volU32*) (adc_baseAddress + 0x30U);
	pADC_Handler->adcReg.pSQR2 = (__volU32*) (adc_baseAddress + 0x34U);
	pADC_Handler->adcReg.pSQR3 = (__volU32*) (adc_baseAddress + 0x38U);
	pADC_Handler->adcReg.pSQR4 = (__volU32*) (adc_baseAddress + 0x3cU);
	pADC_Handler->adcReg.pDR = (__volU32*) (adc_baseAddress + 0x40U);

	pADC_Handler->adcReg.pADCx_CCR = (__volU32*) (adc_baseAddress
			+ (0x300U + 0x08U));
}

void adc_setupAnalogPin(ADC_Handler_t *pADC_Handler, uint8_t pinNumber) {
	uint8_t leastBit = (pinNumber * 2);
	setTwoBitRegister(pADC_Handler->gpioX.pMODER, leastBit, MODE_ANALOG);
}
/**
 * @
 */
void adc_setChannel(ADC_Handler_t *pADC_Handler, uint8_t pcselX, boolean enable) {
	setOneBitRegister(pADC_Handler->adcReg.pPCSEL, pcselX, enable);
}
/**
 * @param channelX -> ADC_PCSELx
 * @param clkCycles -> ADC_CLK_CYCLEx
 */
void adc_setSampleTime(ADC_Handler_t *pADC_Handler, uint8_t pcselX,
		uint8_t clkCycles) {
	uint8_t leastBit = (pcselX < 10) ? (pcselX * 2) : ((pcselX % 10) * 3);
	__volU32 *pSMPRx = 0;
	if (pcselX < 10) {
		pSMPRx = pADC_Handler->adcReg.pSMPR1;
	} else {
		pSMPRx = pADC_Handler->adcReg.pSMPR2;
	}
	setThreeBitRegister(pSMPRx, leastBit, clkCycles);
}

/**
 * @
 */
void adc_setSeq(ADC_Handler_t *pADC_Handler, uint8_t sqrX, uint8_t channelX) {
	__volU32 *pSQRx = 0;
	if (channelX > 0 && channelX <= 4) {
		pSQRx = pADC_Handler->adcReg.pSQR1;
	} else if (channelX > 4 && channelX <= 9) {
		pSQRx = pADC_Handler->adcReg.pSQR2;
	} else if (channelX > 9 && channelX <= 14) {
		pSQRx = pADC_Handler->adcReg.pSQR3;
	} else if (channelX > 15 && channelX <= 16) {
		pSQRx = pADC_Handler->adcReg.pSQR4;
	}
	setForBitRegister(pSQRx, sqrX, channelX);
}

/**
 * @
 */
uint32_t adc_readValue(ADC_Handler_t *pADC_Handler, uint8_t channelX) {
	__volU32 adc_read = 0;
	if (getRegisterValue(pADC_Handler->adcReg.pCR, ADC_CR_ADEN) != TRUE) {
		adc_setADC_On_Off(pADC_Handler, ENABLE);
	}
	setOneBitRegister(pADC_Handler->adcReg.pCR, ADC_CR_ADSTART, ENABLE);
	while (getRegisterValue(pADC_Handler->adcReg.pISR, ADC_ISR_EOC) != TRUE) {
		//BLOCK UNTIL THE AND OF CONERSION
		//	setOneBitRegister(pADC_Handler->adcReg.pISR, ADC_ISR_EOC, TRUE);
	}
	adc_read = *(pADC_Handler->adcReg.pDR);
	return adc_read;
}

/**
 *  SUPPORT FUNCTIONS
 */

uint32_t adc_getADC_baseaddress(uint8_t adcX) {
	switch (adcX) {
	case ADC1:
	case ADC2:
		return ADC1_2_BASEADDRESS;
	case ADC3:
		return ADC3_BASEADDR;
	}
	return 0;
}

/**
 *  TURN ADC ON
 */
void adc_setADC_On_Off(ADC_Handler_t *pADC_Handler, boolean enable) {
	setOneBitRegister(pADC_Handler->adcReg.pCR, ADC_CR_ADEN, ENABLE);
	/*
	if (enable) {
		while (getRegisterValue(pADC_Handler->adcReg.pISR, ADC_ISR_ADRDY)
				== DISABLE) {

		}
	}
	*/
}

void adc_enableTempSensor(ADC_Handler_t *pADC_Handler, boolean enable) {
	setOneBitRegister(pADC_Handler->adcReg.pADCx_CCR, ADCx_CCR_TEMP_SENSOR,
			enable);

	setOneBitRegister(pADC_Handler->adcReg.pADCx_CCR, ADCx_CCR_VOLTAGE_SENSOR,
				enable);
}


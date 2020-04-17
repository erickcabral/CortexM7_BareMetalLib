/*
 * mainADC.c
 *
 *  Created on: Apr 15, 2020
 *      Author: Erick Cabral
 */

#include "adcDriver.h"

int main(void) {

	RCC_Reg_t rcc;
	initializeRCC(&rcc);
	enableGPIOx(&rcc, GPIOB, ENABLE);

	GPIOx_Confg_t gpioB;
	initializeGPIOx(&gpioB, GPIOB);
	gpio_defaultPinOutput(&gpioB, PIN_14);
	gpio_defaultPinOutput(&gpioB, PIN_0);

	ADC_Handler_t adc_Handler;
	adc_initialize(&rcc, &adc_Handler, GPIOB, ADC3);
	adc_enable(&rcc, &adc_Handler, ADC3);

	/*< Setting analog Pin >*/
	adc_setChannel(&adc_Handler, ADC_CHANNEL_19, ENABLE);
	adc_setSampleTime(&adc_Handler, ADC_CHANNEL_19, ADC_CLK_CYCLE_810_5);
	adc_enableTempSensor(&adc_Handler, ENABLE);

	/*< Setting analog Pin >*/
	//adc_setupAnalogPin(&adc_Handler, PIN_10);
	//adc_setChannel(&adc_Handler, ADC_CHANNEL_10, ENABLE);
	//adc_setSampleTime(&adc_Handler, ADC_CHANNEL_10, ADC_CLK_CYCLE_810_5);
	//adc_setSeq(&adc_Handler, ADC_SQR4_SQ10, ADC_CHANNEL_10);

	adc_setADC_On_Off(&adc_Handler, ENABLE);
	while (1) {
		if (getRegisterValue(adc_Handler.adcReg.pCR, ADC_CR_ADEN) == DISABLE) {
			setOneBitRegister(gpioB.pODR, PIN_14, HIGH); //RED LED
			setOneBitRegister(gpioB.pODR, PIN_0, LOW);
		} else {
			setOneBitRegister(gpioB.pODR, PIN_0, HIGH); //GREEN LED
			setOneBitRegister(gpioB.pODR, PIN_14, LOW);
		}
		adc_readValue(&adc_Handler, ADC_CHANNEL_10);
		adc_setADC_On_Off(&adc_Handler, DISABLE);
	}
}

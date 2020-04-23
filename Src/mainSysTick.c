/*
 * mainSysTick.c
 *
 *  Created on: 21 Apr 2020
 *      Author: Erick Cabral
 */

#include "stm32H743xx.h"
#include "gpioDriver.h"

int main(void) {

	RCC_Reg_t rcc;
	rcc_initialize(&rcc);
	rcc_setHSIClock(&rcc,SYS_CLK_16_MHZ);
	rcc_enableGPIOx(&rcc, GPIOB, ENABLE);

	SysTick_Reg_t sTick;
	sysTick_setup(&sTick, CLKSOURCE_INTERNAL, TRUE, SYS_CLK_16_MHZ);

	GPIOx_Confg_t gpioB;
	gpio_initializeGPIOx(&gpioB, GPIOB);
	gpio_defaultPinOutput(&gpioB, PIN_14);

	//	sysTick_startSysTick(&sTick, 1000);
	while (1) {
		sysTick_delayMs(&sTick, 2000);
		if (gpio_getPinState(&gpioB, PIN_14) == LOW) {
			gpio_setPinState(&gpioB, PIN_14, HIGH);
		} else {
			gpio_setPinState(&gpioB, PIN_14, LOW);
		}
	}
}

void SysTick_Handler(void) {
	GPIOx_Confg_t gpioB;
	gpio_initializeGPIOx(&gpioB, GPIOB);
	gpio_defaultPinOutput(&gpioB, PIN_7);
	if (gpio_getPinState(&gpioB, PIN_7) == LOW) {
		gpio_setPinState(&gpioB, PIN_7, HIGH);
	} else {
		gpio_setPinState(&gpioB, PIN_7, LOW);
	}
}

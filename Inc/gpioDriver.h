/*
 * gpioDriver.h
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef GPIODRIVER_H_
#define GPIODRIVER_H_

#include "stm32H743xx.h"

#define GPIOA_BASEADDR				0x58020000
#define GPIOB_BASEADDR				0x58020400
#define GPIOC_BASEADDR				0x58020800



#define GPIOA	0b0000 // 1  Port PA
#define GPIOB	0b0001 // 2  Port PB
#define GPIOC	0b0010 // 3  Port PC
#define GPIOD	0b0011 // 4  Port PD
#define GPIOE	0b0100 // 5  Port PE
#define GPIOF	0b0101 // 6  Port PF
#define GPIOG	0b0110 // 7  Port PG
#define GPIOH	0b0111 // 8  Port PH
#define GPIOI	0b1000 // 9  Port PI
#define GPIOJ	0b1001 // 10 Port PJ
#define GPIOK	0b1010 // 11 Port K

#define PIN_0	0
#define PIN_1	1
#define PIN_2	2
#define PIN_3	3
#define PIN_4	4
#define PIN_5	5
#define PIN_6	6
#define PIN_7	7
#define PIN_8	8
#define PIN_9	9
#define PIN_10	10
#define PIN_11	11
#define PIN_12	12
#define PIN_13	13
#define PIN_14	14
#define PIN_15	15

#define MODE_INPUT				0b00 /* Input mode */
#define MODE_OUTPUT				0b01 /* General purpose output mode */
#define MODE_ALT_FUNC			0b10 /* Alternate function mode */
#define MODE_ANALOG				0b11 /* Analog mode (reset state) */

#define TYPE_PUSH_PULL			0b00 /* 0: Output push-pull (reset state)*/
#define TYPE_OPEN_DRAIN			0b01 /*1: Output open-drain */

#define PUPD_DISABLED			0b00 /* Disabled */
#define PUPD_UP					0b01 /* Resistor UP */
#define PUPD_DOWN				0b10 /* REsistor Down */

#define MODE_INPUT				0b00 /* Low speed */
#define MODE_OUTPUT				0b01 /* Medium speed */
#define MODE_ALT_FUNC			0b10 /* High speed */
#define MODE_ANALOG				0b11 /* Very high speed */

typedef struct {
	__volU32* pMODER;// = (__volU32*) (GPIOx_BASEADDR + 0x00U);
	__volU32* pOTYPER;// = (__volU32*) (GPIOx_BASEADDR + 0x04U);
	__volU32* pOSPEEDR;// = (__volU32*) (GPIOx_BASEADDR + 0x08U);
	__volU32* pPUPDR;// = (__volU32*) (GPIOx_BASEADDR + 0x0CU);
	__volU32* pIDR;// = (__volU32*) (GPIOx_BASEADDR + 0x10U);
	__volU32* pODR;// = (__volU32*) (GPIOx_BASEADDR + 0x14U);
	__volU32* pBSRR;// = (__volU32*) (GPIOx_BASEADDR + 0x18U);
	__volU32* pLCKR;// = (__volU32*) (GPIOx_BASEADDR + 0x1CU);
	__volU32* pAFRL;// = (__volU32*) (GPIOx_BASEADDR + 0x20U);
	__volU32* pAFRH;// = (__volU32*) (GPIOx_BASEADDR + 0x24U);

} GPIOx_Confg_t;

void initializeGPIOx(GPIOx_Confg_t *gpioX, uint32_t gpio_baseAddr);
void setGPIOxModer(GPIOx_Confg_t* gpioX, uint16_t pinNumber,uint8_t pinMode );
void setGPIOxType(GPIOx_Confg_t* gpioX, uint16_t pinNumber,uint8_t pinType );
void setGPIOxSpeed(GPIOx_Confg_t* gpioX, uint16_t pinNumber,uint8_t pinSpeed );
void setGPIOxResistor(GPIOx_Confg_t* gpioX, uint16_t pinNumber,uint8_t pinPUPD );
uint8_t getGPIOxState(GPIOx_Confg_t* gpioX, uint16_t pinNumber);
void setGPIOxOutput(GPIOx_Confg_t* gpioX, uint16_t pinNumber,boolean high_low );
void setGPIOxBSRR(GPIOx_Confg_t* gpioX, uint16_t pinNumber,boolean set_reset );
void setGPIOxLOCK(GPIOx_Confg_t* gpioX, uint16_t pinNumber,boolean locked  );
void setGPIOxALTFunc(GPIOx_Confg_t* gpioX, uint16_t pinNumber,uint16_t altFunc );


#endif /* GPIODRIVER_H_ */

/*
 * adcDriver.h
 *
 *  Created on: 15 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef ADCDRIVER_H_
#define ADCDRIVER_H_

#include "stm32H743xx.h"
#include "gpioDriver.h"

#define ADC1 			1  //AHB1
#define ADC2 			2  //AHB1
#define ADC1_2_BASEADDRESS	0x40022000  //AHB1
#define	ADC3 			3  //AHB4
#define	ADC3_BASEADDR	0x58026000  //AHB4

/**
 * @ISR BITS
 */
#define	ADC_ISR_JQOVF	10
#define	ADC_ISR_AWD3	9
#define	ADC_ISR_AWD2	8
#define	ADC_ISR_AWD1	7
#define	ADC_ISR_JEOS	6
#define	ADC_ISR_JEOC	5
#define	ADC_ISR_OVR		4
#define	ADC_ISR_EOS		3
#define	ADC_ISR_EOC		2
#define	ADC_ISR_EOSMP	1
#define	ADC_ISR_ADRDY	0

/**
 * @IER BITS
 */
#define ADC_IER_JQOVFIE		10
#define ADC_IER_AWD3IE		9
#define ADC_IER_AWD2IE		8
#define ADC_IER_AWD1IE		7
#define ADC_IER_JEOSIE		6
#define ADC_IER_JEOCIE		5
#define ADC_IER_OVRIE		4
#define ADC_IER_EOSIE		3
#define ADC_IER_EOCIE		2
#define ADC_IER_EOSMPIE		1
#define ADC_IER_ADRDYIE		0

/**
 * @IER BITS
 */
#define ADC_CR_ADCAL			31
#define ADC_CR_ADCALDIF			30
#define ADC_CR_DEEPPWD			29
#define ADC_CR_ADVREGEN			28
#define ADC_CR_LINCALRDYW6		27
#define ADC_CR_LINCALRDYW5		26
#define ADC_CR_LINCALRDYW4		25
#define ADC_CR_LINCALRDYW3		24
#define ADC_CR_LINCALRDYW2		23
#define ADC_CR_LINCALRDYW1		22
#define ADC_CR_ADCALLIN			16
#define ADC_CR_BOOST			8 //[9:8]
#define ADC_CR_JADSTP			5
#define ADC_CR_ADSTP			4
#define ADC_CR_JADSTART			3
#define ADC_CR_ADSTART			2
#define ADC_CR_ADDIS			1
#define ADC_CR_ADEN				0

/**
 * @ADC_SMPR1 BITS
 */
#define ADC_SMPR1_SMP0		0
#define ADC_SMPR1_SMP1		3
#define ADC_SMPR1_SMP2		6
#define ADC_SMPR1_SMP3		9
#define ADC_SMPR1_SMP4		12
#define ADC_SMPR1_SMP5		15
#define ADC_SMPR1_SMP6		18
#define ADC_SMPR1_SMP7		21
#define ADC_SMPR1_SMP8		24
#define ADC_SMPR1_SMP9		27

/**
 * @ADC_SMPR2 BITS
 */
#define ADC_SMPR1_SMP10		0
#define ADC_SMPR1_SMP11		3
#define ADC_SMPR1_SMP12		6
#define ADC_SMPR1_SMP13		9
#define ADC_SMPR1_SMP14		12
#define ADC_SMPR1_SMP15		15
#define ADC_SMPR1_SMP16		18
#define ADC_SMPR1_SMP17		21
#define ADC_SMPR1_SMP18		24
#define ADC_SMPR1_SMP19		27

/**
 * @ADC_SMPR2 BITS
 */
#define ADC_PCSEL_PCSEL0	0
#define ADC_PCSEL_PCSEL1	1
#define ADC_PCSEL_PCSEL2	2
#define ADC_PCSEL_PCSEL3	3
#define ADC_PCSEL_PCSEL4	4
#define ADC_PCSEL_PCSEL5	5
#define ADC_PCSEL_PCSEL6	6
#define ADC_PCSEL_PCSEL7	7
#define ADC_PCSEL_PCSEL8	8
#define ADC_PCSEL_PCSEL9	9
#define ADC_PCSEL_PCSEL10	10
#define ADC_PCSEL_PCSEL11	11
#define ADC_PCSEL_PCSEL12	12
#define ADC_PCSEL_PCSEL13	13
#define ADC_PCSEL_PCSEL14	14
#define ADC_PCSEL_PCSEL15	15
#define ADC_PCSEL_PCSEL16	16
#define ADC_PCSEL_PCSEL17	17
#define ADC_PCSEL_PCSEL18	18
#define ADC_PCSEL_PCSEL19	19

/**
 * @ADC_SQR1 BITS
 */
#define ADC_SQR1_L			0
#define ADC_SQR1_SQ1		6
#define ADC_SQR1_SQ2		12
#define ADC_SQR1_SQ3		18
#define ADC_SQR1_SQ4		24
/**
 * @ADC_SQR2 BITS
 */
#define ADC_SQR2_SQ5		0
#define ADC_SQR2_SQ6		6
#define ADC_SQR2_SQ7		12
#define ADC_SQR2_SQ8		18
#define ADC_SQR2_SQ9		24
/**
 * @ADC_SQR3 BITS
 */
#define ADC_SQR3_SQ5		0
#define ADC_SQR3_SQ6		6
#define ADC_SQR3_SQ7		12
#define ADC_SQR3_SQ8		18
#define ADC_SQR3_SQ9		24
/**
 * @ADC_SQR2 BITS
 */
#define ADC_SQR4_SQ10		0
#define ADC_SQR4_SQ11		6
#define ADC_SQR4_SQ12		12
#define ADC_SQR4_SQ13		18
#define ADC_SQR4_SQ14		24

/**
 * @ADCx_CCR BITS
 */
#define ADCx_CCR_VOLTAGE_SENSOR		22
#define ADCx_CCR_TEMP_SENSOR		23
#define ADCx_CCR_BAT_SENSOR			24

typedef struct {
	__volU32 *pISR;		//OFFSET 0x00U
	__volU32 *pIER;		//OFFSET 0x04U
	__volU32 *pCR;		//OFFSET 0x08U
	__volU32 *pCFGR;	//OFFSET 0x0CU
	__volU32 *pCFGR2;	//OFFSET 0x10U
	__volU32 *pSMPR1;	//OFFSET 0x14U
	__volU32 *pSMPR2;	//OFFSET 0x18U
	__volU32 *pPCSEL;	//OFFSET 0x1CU
	__volU32 *pSQR1;	//OFFSET 0x30U
	__volU32 *pSQR2;	//OFFSET 0x34U
	__volU32 *pSQR3;	//OFFSET 0x38U
	__volU32 *pSQR4;	//OFFSET 0x3cU
	__volU32 *pDR;		//OFFSET 0x40U

	__volU32 *pADCx_CCR; //OFFSET  (0x300U + 0x08U)
} ADC_Reg_t;

typedef struct {
	GPIOx_Confg_t gpioX;
	ADC_Reg_t adcReg;
} ADC_Handler_t;

/**
 * @
 */
void adc_enable(RCC_Reg_t *pRCC, uint8_t adcX);

/**
 * @
 */
void adc_initialize(ADC_Handler_t *pADC_Handler, uint8_t gpioX, uint8_t adcX);

/**
 * @
 */
void adc_setupAnalogPin(ADC_Handler_t *pADC_Handler, uint8_t pinNumber);

/**
 * @
 */
void adc_setChannel(ADC_Handler_t *pADC_Handler, uint8_t channelX,
		boolean enable);

/**
 * @ADC_SAMPLER CLCK CYCLES
 */
#define ADC_CLK_CYCLE_1_5		0b000 // 1.5 ADC clock cycles
#define ADC_CLK_CYCLE_2_5		0b001 // 2.5 ADC clock cycles
#define ADC_CLK_CYCLE_8_5		0b010 // 8.5 ADC clock cycles
#define ADC_CLK_CYCLE_16_5		0b011 // 16.5 ADC clock cycles
#define ADC_CLK_CYCLE_32_5		0b100 // 32.5 ADC clock cycles
#define ADC_CLK_CYCLE_64_5		0b101 // 64.5 ADC clock cycles
#define ADC_CLK_CYCLE_387_5		0b110 // 387.5 ADC clock cycles
#define ADC_CLK_CYCLE_810_5		0b111 // 810.5 ADC clock cycles

void adc_setSampleTime(ADC_Handler_t *pADC_Handler, uint8_t channelX,
		uint8_t clkCycles);
/**
 * @
 */
void adc_setSeq(ADC_Handler_t *pADC_Handler, uint8_t sqrX, uint8_t channelX);

/**
 * @
 */
uint32_t adc_readValue(ADC_Handler_t *pADC_Handler, uint8_t pinNumber);

/**
 *  SUPPORT FUNCTIONS
 */

uint32_t adc_getADC_baseaddress(uint8_t adcX);

void adc_setADC_On_Off(ADC_Handler_t *pADC_Handler, boolean enable);
/*!< TEMP SENSOR >*/

void adc_enableTempSensor(ADC_Handler_t *pADC_Handler, boolean enable);

#endif /* ADCDRIVER_H_ */

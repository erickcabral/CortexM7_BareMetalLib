/*
 * stm32H743xx.h
 *
 *  Created on: 9 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef STM32H743XX_H_
#define STM32H743XX_H_

#include <stdint-gcc.h>

/*<! COMMOM VARIABLES >*/
#define __uint8				uint8_t
#define __int32U			uint32_t

#define __volU16			volatile uint16_t
#define __volU32			volatile uint32_t

#define boolean				__uint8

#define TRUE				1
#define FALSE				0
#define ENABLE				TRUE
#define DISABLE				FALSE

#define HIGH				TRUE
#define LOW					FALSE

/****************** SYSTEM CONSTANTS ******************************/
#define SYS_CLK_64_MHZ  	64 	//00: Division by 1, hsi(_ker)_ck = 64 MHz (default after reset)
#define SYS_CLK_32_MHZ  	32 	//01: Division by 2, hsi(_ker)_ck = 32 MHz
#define SYS_CLK_16_MHZ  	16 	//10: Division by 4, hsi(_ker)_ck = 16 MHz
#define SYS_CLK_8_MHZ  		8	//11: Division by 8, hsi(_ker)_ck = 8 MHz




/*******************************************************************/

#define RCC_BASEADDR		0x58024400U

#define APB1LENR 			(RCC_BASEADDR + 0x0E8U)
#define C1_APB1LENR 		(RCC_BASEADDR + 0x148U)

#define APB2ENR				(RCC_BASEADDR + 0x0F0U)
#define C1_APB2ENR			(RCC_BASEADDR + 0x150U)

/**
 * @CR REGISTER BITS
 */

#define RCC_CR_HSIRDY		2
#define RCC_CR_HSIKERON		1
#define RCC_CR_HSION		0

/**
 * @RCC_AHB4ENR BITS
 */

#define RCC_AHB4ENR_ADC3EN		24

/**
 * @RCC_AHB1ENR BITS
 */
#define RCC_AHB4ENR_ADC12EN		5

/**
 * @RCC_D3CCIPR BITS
 */
#define RCC_D3CCIPR_ADCSEL		16

/*****************************************************************************
 * @RCC REGISTER CONFIG API
 */
typedef struct {

	__volU32 *pCR; 		//OFFSET 0x00U

	__volU32 *pAHB1ENR; 		//OFFSET 0x0D8U
	__volU32 *pC1_AHB1ENR; 		//OFFSET 0x138U

	__volU32 *pAHB4ENR; 		// = (__volU32*)(RCC_BASEADDR + 0x0E0U);
	__volU32 *pC1_AHB4ENR; 		// = (__volU32*)(RCC_BASEADDR + 0x140U);

	__volU32 *pAPB4ENR; 		// = (__volU32*)(RCC_BASEADDR + 0x0F4U);
	__volU32 *pC1_APB4ENR; 		// = (__volU32*)(RCC_BASEADDR + 0x154U);

	__volU32 *pAPB1LENR; 		// = (__volU32*)(RCC_BASEADDR + 0x0E8U);
	__volU32 *pC1_APB1LENR; 	// = (__volU32*)(RCC_BASEADDR + 0x148U);

	__volU32 *pAPB2ENR; 		// OFFSET 0x0F0U;
	__volU32 *pC1_APB2ENR; 		// OFFSET 0x150U;

	__volU32 *pD3CCIPR; 		// OFFSET 0x058U

} RCC_Reg_t;

void rcc_initialize(RCC_Reg_t *pRCC);
void rcc_setHSIClock(RCC_Reg_t *pRCC, uint8_t sys_clk_xMhz);
void rcc_enableSYSCFG(RCC_Reg_t *pRCC, boolean enable);
void rcc_enableGPIOx(RCC_Reg_t *pRCC, uint8_t gpioX, boolean enable);

/**************************** CRS REGISTER CONFIG API ******************************
 ************************* CLOCK RECOVERY SYSTEM (SysTick) *************************/

#define SYS_TICK_CALIB		0x3E8	//Default SYS_TICK_CLOCK Calibration Value.
/*************************************
 * @CRS_CR Register Bits
 */
#define CRS_CR_TRIM			8
#define CRS_CR_SWSYNC		7
#define CRS_CR_AUTOTRIMEN	6
#define CRS_CR_CEN			5
#define CRS_CR_ESYNCIE		3
#define CRS_CR_ERRIE		2
#define CRS_CR_SYNCWARNIE	1
#define CRS_CR_SYNCOKIE		0

/*************************************
 * @CRS_CFGR Register Bits
 */
#define CRS_CFGR_SYNC_POL	31
#define CRS_CFGR_SYNC_SRC	28
#define CRS_CFGR_SYNC_DIV	24
#define CRS_CFGR_FELIM		16
#define CRS_CFGR_RELOAD		0
/*************************************
 * @CRS_ISR Register Bits
 */
#define CRS_ISR_FECAP		16
#define CRS_ISR_FEDIR		15
#define CRS_ISR_TRIMOVF		10
#define CRS_ISR_SYNCMISS	9
#define CRS_ISR_SYNCERR		8
#define CRS_ISR_ESYNCF		3
#define CRS_ISR_ERRF		2
#define CRS_ISR_SYNCWARNF	1
#define CRS_ISR_SYNCOKF		0
/*************************************
 * @CRS_ICR Interrupt Clear Register Register Bits
 */
#define CRS_ISR_ESYNCC 		4
#define CRS_ISR_ERRC	 	3
#define CRS_ISR_SYNC_WARNC	1
#define CRS_ISR_SYNC_OKC	0

typedef struct {
	__volU32 *pCR; 		//OFFSET 0x00U
	__volU32 *pCFGR; 	//OFFSET 0x04U
	__volU32 *pISR; 	//OFFSET 0x08U
	__volU32 *pICR; 	//OFFSET 0x0CU

} CSR_Reg_t;

#define TIMER_COUNT_DOWN  	0
#define TIMER_COUNT_UP 		1

/**************************** SYSTEM TIMER (SysTick) CONFIG API ******************************
 ************************* CLOCK RECOVERY SYSTEM (SysTick) *************************/
/*
 #define pSYST_CSR 		(__volU32*)0xE000E010	// Privileged 0x00000004 SysTick Control and Status Register
 #define pSYST_RVR 		(__volU32*)0xE000E014	// RW Privileged UNKNOWN SysTick Reload Value Register on page 4-34
 #define pSYST_CVR 		(__volU32*)0xE000E018	// RW Privileged UNKNOWN SysTick Current Value Register on page 4-35
 #define pSYST_CALIB 	(uint32_t*)0xE000E01C 	//RO Privileged 0xC0000000 SysTick Calibration Value Register on page 4-35
 */
/*************************************
 * @ SYST_CSR Register Bits
 */
#define SYST_CSR_COUNTFLAG 		16
#define SYST_CSR_CLKSOURCE		2
#define SYST_CSR_TICKINT		1
#define SYST_CSR_ENABLE			0

#define CLKSOURCE_EXTERNAL		0
#define CLKSOURCE_INTERNAL		1
/*************************************
 * @ SYST_RVR Register Bits
 */
#define SYST_RVR_RELOAD			0
/*************************************
 * @ SYST_CVR Register Bits
 */
#define SYST_CVR_CURRENT		0
/*************************************
 * @ SYST_CALIB Register Bits
 */
#define SYST_CALIB_NOREF	31
#define SYST_CALIB_SKEW		30
#define SYST_CALIB_TENMS	0
/*************************************
 * @ SYST_CALIB Register Bits
 */
typedef struct {
	uint16_t systemClockMhz;
	__volU32 *pCSR;
	__volU32 *pRVR;
	__volU32 *pCVR;
	uint32_t *pCALIB;
} SysTick_Reg_t;

/********************* SYSTICK FUNCTIONS ********************************/
void sysTick_setup(SysTick_Reg_t *pSysTick,
		boolean clock_source, boolean genException, uint8_t sysClockMhz) ;

void sysTick_startSysTick(SysTick_Reg_t *pSysTick, uint16_t timerMillis);

void sysTick_stopSysTick(SysTick_Reg_t *pSysTick);

void sysTick_delayMs(SysTick_Reg_t *pSysTick, uint16_t millis);

boolean sysTick_getState(SysTick_Reg_t *pSysTick);

/********************************************* SUPPORT FUCNTIONS **************************************************/
void resetRegister(__volU32 *pRegister, uint8_t leastBit, uint8_t numberOfBits);
void setRegister(__volU32 *pRegister, uint32_t value, uint8_t leastBit,
		uint8_t numOfBits);

void setOneBitRegister(__volU32 *pRegister, uint8_t leastBit, boolean enable);
void setTwoBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue);
void setThreeBitRegister(__volU32 *pRegister, uint8_t leastBit,
		uint16_t newValue);
void setForBitRegister(__volU32 *pRegister, uint8_t leastBit, uint16_t newValue);

boolean getRegisterValue(__volU32 *pRegister, uint8_t bitToRead);

/********************************************* OUTROS **************************************************/

#define pTEMPER_SENSOR_CAL1			(__volU32*)0x1FF1E820
#define pTEMPER_SENSOR_CAL2			(__volU32*)0x1FF1E840

#endif /* STM32H743XX_H_ */

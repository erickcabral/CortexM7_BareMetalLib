/*
 * GPTIM_2_5_Driver.h
 *
 *	GENERAL PURPOSE TIMERS 2/3/4/5 API
 *
 *  Created on: 20 Apr 2020
 *      Author: Erick Cabral
 */

#ifndef GPTIM2_5_DRIVER_H_
#define GPTIM2_5_DRIVER_H_

#include "stm32H743xx.h"

/*****************************
 * @TIMx_CR1 Register bits
 */
#define TIMx_CR1_UIFREMAP	11
#define TIMx_CR1_CKD		8
#define TIMx_CR1_ARPE		7
#define TIMx_CR1_CMS		5
#define TIMx_CR1_DIR		4
#define TIMx_CR1_OPM		3
#define TIMx_CR1_URS		2
#define TIMx_CR1_UDIS		1
#define TIMx_CR1_CEN		0

/*****************************
 * @TIMx_CR2 Register bits
 */
#define TIMx_CR2_TI1S	7
#define TIMx_CR2_ MMS	4
#define TIMx_CR2_CCDS	3

/*****************************
 * @TIMx_CNT Register bits
 */
#define TIMx_CNT_3		31
#define TIMx_CNT_2		16
#define TIMx_CNT_1		0

/*****************************
 * @TIMx_CNT Register bits
 */
#define TIMx_CNT_UIFCPY		31
#define TIMx_CNT_2			16
#define TIMx_CNT_1			0

typedef struct{
	__volU32 CR1;		//OFFSET 0x00U
	__volU32 CR2;		//OFFSET 0x04U

	__volU32 CNT;		//OFFSET 0x24U
	__volU32 PSC;		//OFFSET 0x28U
	__volU32 ARR;		//OFFSET 0x2CU

	__volU32 CCR1;		//OFFSET 0x34U
	__volU32 CCR2;		//OFFSET 0x38U
	__volU32 CCR3;		//OFFSET 0x3CU
	__volU32 CCR4;		//OFFSET 0x40U

}TIMERS_2to5_Reg_t;

#endif /* GPTIM2_5_DRIVER_H_ */

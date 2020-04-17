/*
 * irqDriver.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Erick Cabral
 */
#include "stm32H743xx.h"
#include "irqDriver.h"

void enableIRQ(__uint8 irqNum, boolean enable) {
	__uint8 placeHolder = irqNum / 32;
	uint8_t bitShift =
			placeHolder == 0 ? irqNum : (irqNum % (32 * placeHolder));

	__volU32 *pISERx =
			((__volU32*) (NVIC_pISER0_BASEADDRESS + (placeHolder * 4)));
	__volU32 *pICERx =
			((__volU32*) (NVIC_pICER0_BASEADDRESS + (placeHolder * 4)));

	if (enable) {
		*pISERx |= (1 << bitShift);
	} else {
		*pICERx |= (1 << bitShift);
	}
}

void setIRQPenReg(__uint8 irqNum, boolean enable) {
	__uint8 placeHolder = irqNum / 32;
	uint8_t bitShift =
			placeHolder == 0 ? irqNum : (irqNum % (32 * placeHolder));
	__volU32 *pISPRx = ((__volU32*) (NVIC_pISPR0_BASEADDR + (placeHolder * 4)));
	__volU32 *pICPRx = ((__volU32*) (NVIC_pICPR0_BASEADDR + (placeHolder * 4)));

	if (enable) {
		*pISPRx |= (1 << bitShift);
	} else {
		*pICPRx |= (1 << bitShift);
	}
}
void setIRQ_PRIORITY(uint16_t irqNum, uint8_t priority) {
	uint8_t placeHolder = irqNum / 4; // PRx Register Place
	uint8_t bitShifter = (irqNum == 0 ? irqNum : (irqNum % (placeHolder * 4)));
	uint8_t validBitsShifter = (bitShifter + 4);
	__volU32 *pPRx = ((__volU32*) (NVIC_pPRI0_BASEADDR + (placeHolder * 4)));

	*(pPRx) |= (priority << validBitsShifter);

}

/*<! SYSCFG CONFIG >*/
/**
 *  PIN EDGE DETECTOR TYPES:
 *  DISABLED	(0)
 *  RISING_EDGE (1)
 *  FALLING_EDGE(2)
 *  BOTH_EDGES 	(3)
 */
void setPinEdgeDetector(EXTI_Reg_t *pEXTI, uint8_t pinNumber, uint8_t edgeType) {
	pEXTI->pRTSR1 = (__volU32*) (EXTI_RTSR_BASEADDR);
	pEXTI->pFTSR1 = (__volU32*) (EXTI_FTSR1_BASEADDR);
	switch (edgeType) {
	case 0:
		setOneBitRegister(pEXTI->pRTSR1, pinNumber, DISABLE);
		setOneBitRegister(pEXTI->pFTSR1, pinNumber,DISABLE);
		break;
	case 1:
		setOneBitRegister(pEXTI->pRTSR1, pinNumber, ENABLE);
		setOneBitRegister(pEXTI->pFTSR1, pinNumber, DISABLE);
		break;
	case 2:
		setOneBitRegister(pEXTI->pRTSR1, pinNumber, DISABLE);
		setOneBitRegister(pEXTI->pFTSR1, pinNumber, ENABLE);
		break;
	case 3:
		setOneBitRegister(pEXTI->pRTSR1, pinNumber, ENABLE);
		setOneBitRegister(pEXTI->pFTSR1, pinNumber, ENABLE);
		break;
	}
}

/*
 void setRTSR(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable) {
 pEXTI->pRTSR1 = (__volU32*) (EXTI_RTSR_BASEADDR);
 pEXTI->pFTSR1 = (__volU32*) (EXTI_FTSR1_BASEADDR);
 setOneBitRegister(pEXTI->pRTSR1, pinNumber, enable);
 setOneBitRegister(pEXTI->pFTSR1, pinNumber, DISABLE);
 }
 void setFTSR(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable) {
 pEXTI->pRTSR1 = (__volU32*) (EXTI_RTSR_BASEADDR);
 pEXTI->pFTSR1 = (__volU32*) (EXTI_FTSR1_BASEADDR);
 setOneBitRegister(pEXTI->pRTSR1, pinNumber, DISABLE);
 setOneBitRegister(pEXTI->pFTSR1, pinNumber, enable);
 }
 void setRTandFT(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable) {
 pEXTI->pRTSR1 = (__volU32*) (EXTI_RTSR_BASEADDR);
 pEXTI->pFTSR1 = (__volU32*) (EXTI_FTSR1_BASEADDR);
 setOneBitRegister(pEXTI->pRTSR1, pinNumber, enable);
 setOneBitRegister(pEXTI->pFTSR1, pinNumber, enable);
 }
 */

void setEXTI_CR(SYSCFG_Reg_t *pSYSCFG, uint8_t pinNumber, uint16_t gpioX) {
	uint8_t placeHolder = pinNumber / 4;
	uint8_t bitShifter =
			placeHolder == 0 ?
					(pinNumber * 4) : ((pinNumber % (placeHolder * 4)) * 4);
	__volU32 *EXTICRx_ADDRESS = (uint32_t*) (pEXTICR0_BASEADDRESS
			+ (placeHolder));
	pSYSCFG->pEXTICR[placeHolder] = EXTICRx_ADDRESS;
	*(pSYSCFG->pEXTICR[placeHolder]) |= (gpioX << bitShifter);
}

/*<! EXTI CONFIG API >*/
void enableCPUIMRx(EXTI_Reg_t *pEXTI, uint8_t imrX, uint8_t maskNum,
boolean enable) {
	__volU32 *CPUIMRx_ADDRESS = ( pEXTI_CPUIMRx_BASEADDR + (imrX));
	pEXTI->pCPUIMR[imrX] = CPUIMRx_ADDRESS;
	if (enable) {
		*(pEXTI->pCPUIMR[imrX]) |= (1 << maskNum);
	} else {
		*(pEXTI->pCPUEMR[imrX]) |= (1 << maskNum);
	}
}

void setCPUEMRx();

void setCPUIPRx(EXTI_Reg_t *pEXTI, uint8_t imrX, uint8_t maskNum,
boolean enable) {
	switch (imrX) {
	case 0:
		pEXTI->pCPUPR[imrX] = ((__volU32*) (EXTI_BASEADDR + 0x88U));
		break;
	case 1:
		pEXTI->pCPUPR[imrX] = ((__volU32*) (EXTI_BASEADDR + 0x98U));
		break;
	case 2:
		pEXTI->pCPUPR[imrX] = ((__volU32*) (EXTI_BASEADDR + 0xA8U));
		break;
	}

	if (enable) {
		setOneBitRegister(pEXTI->pCPUPR[imrX], maskNum, enable);
	} else {
		boolean pr = (*(pEXTI->pCPUPR[imrX]) >> maskNum);
		if (pr == ENABLE) {
			setOneBitRegister(pEXTI->pCPUPR[imrX], maskNum, DISABLE);
		}
	}
}

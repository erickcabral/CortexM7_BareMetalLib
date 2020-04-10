/*
 * irqDriver.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Erick Cabral
 */

#ifndef IRQDRIVER_H_
#define IRQDRIVER_H_

#include <stdint-gcc.h>

#define __uint8				uint8_t	//unsigned short int
#define __int32U			uint32_t //	unsigned long int

#define __volU16			volatile uint16_t //	volatile unsigned short int
#define __volU32			volatile uint32_t //	volatile unsigned long int

#define boolean				__uint8

#define TRUE				1
#define FALSE				0

/*<! NVIC IRQ BASE ADRESSES >*/

#define NVIC_pISER0_BASEADDRESS			0xE000E100U
/*
 #define NVIC_pISER0						((__volU32*)0xE000E100U)
 #define NVIC_pISER1						((__volU32*)0xE000E104U)
 #define NVIC_pISER2						((__volU32*)0xE000E108U)
 #define NVIC_pISER3						((__volU32*)0xE000E10CU)
 #define NVIC_pISER4						((__volU32*)0xE000E110U)
 #define NVIC_pISER5						((__volU32*)0xE000E114U)
 #define NVIC_pISER6						((__volU32*)0xE000E118U)
 #define NVIC_pISER7						((__volU32*)0xE000E11CU)
 */
#define NVIC_pICER0_BASEADDRESS			0XE000E180U
/*
 #define NVIC_pICER0						((__volU32*)0XE000E180U)
 #define NVIC_pICER1						((__volU32*)0XE000E184U)
 #define NVIC_pICER2						((__volU32*)0XE000E188U)
 #define NVIC_pICER3						((__volU32*)0XE000E18CU)
 #define NVIC_pICER4						((__volU32*)0XE000E190U)
 #define NVIC_pICER5						((__volU32*)0XE000E194U)
 #define NVIC_pICER6						((__volU32*)0XE000E198U)
 #define NVIC_pICER7						((__volU32*)0XE000E19CU)
 */
#define NVIC_pISPR0_BASEADDR			0XE000E200U
/*
 #define NVIC_pISPR0						((__volU32*)0XE000E200U)
 #define NVIC_pISPR1						((__volU32*)0XE000E204U)
 */
#define NVIC_pICPR0_BASEADDR			0XE000E280U
/*
 #define NVIC_pICPR0						((__volU32*)0XE000E280U)
 #define NVIC_pICPR1						((__volU32*)0XE000E284U)
 */

#define NVIC_pPRI0_BASEADDR				0xE000E400U

/*
 #define NVIC_pPRI0						((__volU32*)0xE000E400U)
 */
//void enableIRQ(__uint8 irqNum, boolean enable);
/*
 typedef struct {
 __volU32 *pISERx; //= NVIC_ISER0_BASEADDR;
 __volU32 *pICERx; //= NVIC_ICER0_BASEADDR;
 __volU32 *pISPRx; //= NVIC_ISPR0_BASEADDR;
 __volU32 *pPRIx; //= NVIC_PRI0_BASEADDR;
 //void enableIRQ(__uint8 irqNum, boolean enable);
 //void setIRQPenReg(__uint8 irqNum, boolean enable);
 //void setIRQ_PRIORITY(__uint8 irqNum);
 } NVIC_Reg_t;
 */
void enableIRQ(__uint8 irqNum, boolean enable);
void setIRQPenReg(__uint8 irqNum, boolean enable);
void setIRQ_PRIORITY(uint16_t irqNum, uint8_t priority);

/*<! SYSCFG CONFIG API >*/

#define SYSCFG_BASEADDR				0x58000400U
#define pEXTICR0_BASEADDRESS		(uint32_t*)0x58000408U

typedef struct {
	__volU32 *pPMCR;		// OFFSET 0x04U
	__volU32 *pEXTICR[4];		// OFFSET 0x08U
//__volU32 *pEXTICR2;		// OFFSET 0x0CU
//__volU32 *pEXTICR3;		// OFFSET 0x10U
//__volU32 *pEXTICR4;		// OFFSET 0x14U
} SYSCFG_Reg_t;

void setEXTI_CR(SYSCFG_Reg_t *pSYSCFG, uint8_t pinNumber, uint16_t gpioX);

/*<! EXTI CONFIG API   >*/

#define EXTI_BASEADDR							0x58000000U
#define EXTI_RTSR_BASEADDR						((uint32_t*)EXTI_BASEADDR)
#define EXTI_FTSR1_BASEADDR						((uint32_t*)0x58000004U)
#define pEXTI_CPUIMRx_BASEADDR					((__volU32*)0x58000080U)
#define EXTI_CPUEMRx_BASEADDR					((__volU32*)0x58000084U)
#define EXTI_CPUPRx_BASEADDR					((__volU32*)0x58000088U)

#define IMR1				0
#define IMR2				1
#define IMR3				2


#define DISABLED 		DISABLE
#define RISING_EDGE 	1
#define FALLING_EDGE 	2
#define BOTH_EDGES 		3


typedef struct {
	__volU32 *pRTSR1;		// OFFSET 0x00U
	__volU32 *pFTSR1;	// OFFSET 0x04U
	//..
	__volU32 *pCPUIMR[3];		// OFFSET 0x80CU
	__volU32 *pCPUEMR[3];		// OFFSET 0x84U
	__volU32 *pCPUPR[3];		// OFFSET 0x88U
//..
} EXTI_Reg_t;

void setPinEdgeDetector(EXTI_Reg_t *pEXTI, uint8_t pinNumber, uint8_t edgeType);
void setRTSR(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable);
void setFTSR(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable);
void setRTandFT(EXTI_Reg_t *pEXTI, uint8_t pinNumber, boolean enable);

void enableCPUIMRx(EXTI_Reg_t *pEXTI, uint8_t imrX, uint8_t maskNum,
boolean enable);
void setCPUEMRx();
void setCPUIPRx(EXTI_Reg_t *pEXTI, uint8_t imrX, uint8_t maskNum,
boolean enable);

#endif /* IRQDRIVER_H_ */

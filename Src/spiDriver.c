/*
 * spiDriver.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Erick Cabral
 */

#include "spiDriver.h"

void spi_initializeDriver(SPI_ConfigHandler_t *pSPI_Handler, uint8_t gpioX,
		uint8_t spiX) {
	initializeGPIOx(&pSPI_Handler->pGPIOx, gpioX);

	uint32_t spiX_baseaddress = spi_GetSPIxBaseaddress(spiX);
	pSPI_Handler->pSPI.pCR1 = (__volU32*) (spiX_baseaddress + 0x00U);
	pSPI_Handler->pSPI.pCR2 = (__volU32*) (spiX_baseaddress + 0x04U);
	pSPI_Handler->pSPI.pCFG1 = (__volU32*) (spiX_baseaddress + 0x08U);
	pSPI_Handler->pSPI.pCFG2 = (__volU32*) (spiX_baseaddress + 0x0CU);
	pSPI_Handler->pSPI.pIER = (__volU32*) (spiX_baseaddress + 0x10U);
	pSPI_Handler->pSPI.pSR = (__volU32*) (spiX_baseaddress + 0x14U);
	pSPI_Handler->pSPI.pIFCR = (__volU32*) (spiX_baseaddress + 0x18U);
	pSPI_Handler->pSPI.pTXDR = (__volU32*) (spiX_baseaddress + 0x20U);
	pSPI_Handler->pSPI.pRXDR = (__volU32*) (spiX_baseaddress + 0x30U);
	pSPI_Handler->pSPI.pCRCPOLY = (__volU32*) (spiX_baseaddress + 0x40U);
	pSPI_Handler->pSPI.pTXCRC = (__volU32*) (spiX_baseaddress + 0x44U);
	pSPI_Handler->pSPI.pRXCRC = (__volU32*) (spiX_baseaddress + 0x48U);
	pSPI_Handler->pSPI.pUDRDR = (__volU32*) (spiX_baseaddress + 0x4CU);
	pSPI_Handler->pSPI.pI2SCFGR = (__volU32*) (spiX_baseaddress + 0x50U);

}

void spi_enableSPIx(RCC_Reg_t *pRCC, uint8_t spiX) {
	uint8_t spiBitPosition;

	switch (spiX) {
	case SPI1: //12
	case SPI4: //13
	case SPI5: //20
		if (spiX == SPI1) {
			spiBitPosition = 12;
		} else if (spiX == SPI2) {
			spiBitPosition = 13;
		} else {
			spiBitPosition = 20;
		}
		*(pRCC->pC1_APB2ENR) |= (1 << spiBitPosition);
		break;
	case SPI2:
	case SPI3:
		//APB1
		break;
	case SPI6:
		//APB4
		break;
	}
}

uint32_t spi_GetSPIxBaseaddress(uint8_t spiX) {
	switch (spiX) {
	case SPI1:
		return (SPI1_BASEADDRESS);
	case SPI2:
		return (SPI2_BASEADDRESS);
	}
	return 0x0;
}

void spi_setupMasterPins(SPI_ConfigHandler_t *pSPI_Handler,
		uint8_t mosiPinNumber, uint8_t misoPinNumber, uint8_t sclkPinNumber,
		uint8_t nssPinNumber) {

	uint8_t gpioPinNumber[4];
	gpioPinNumber[0] = mosiPinNumber;
	gpioPinNumber[1] = misoPinNumber;
	gpioPinNumber[2] = sclkPinNumber;
	gpioPinNumber[3] = nssPinNumber;

	uint8_t leastBit = 0;
	uint8_t pinNumber = 0;
	for (int i = 0; i < 4; i++) {
		pinNumber = gpioPinNumber[i];
		switch (i) {
		case 0: //SETUP MOSI
		case 2:	//SETUP SCLK
		case 3:	//SETUP NSS
			leastBit = 2 * pinNumber;
			setTwoBitRegister(pSPI_Handler->pGPIOx.pMODER, leastBit,
					MODE_OUTPUT);	//MODE_OUTPUT
			//setTwoBitRegister(pSPI_ConfigHandler->spiGPIO->pMODER, leastBit,	MODE_OUTPUT	);	//MODE_OUTPUT
			break;
		case 1:	//SETUP MISO
			leastBit = 2 * pinNumber;
			setTwoBitRegister(pSPI_Handler->pGPIOx.pMODER, leastBit,
					MODE_INPUT);	//MODE_OUTPUT
			break;
		}
		setOneBitRegister(pSPI_Handler->pGPIOx.pOTYPER, pinNumber,
				TYPE_PUSH_PULL); //TYPE
		leastBit = 2 * pinNumber;
		setTwoBitRegister(pSPI_Handler->pGPIOx.pOSPEEDR, leastBit, SPEED_HIGH);	//SPEED
		setTwoBitRegister(pSPI_Handler->pGPIOx.pPUPDR, leastBit, PUPD_UP);//PUPDR
	}
}
/**
 *  spiMode 	-> MASTER / SLAVE
 *  commTYPE 	-> HALF/FULL DUPLEX, RX/TX SIMPLEX
 *  cpolType	->
 *  cphaType	->
 *  baundRate	->
 */
void spi_configSPI(SPI_ConfigHandler_t *pSPI_Handler, uint8_t spiMode,
		uint8_t commType, uint8_t cpolType, uint8_t cphaType, uint8_t baundRate) {
	setOneBitRegister(pSPI_Handler->pSPI.pCFG2, SPI_CFG2_CPOL, cpolType);
	setOneBitRegister(pSPI_Handler->pSPI.pCFG2, SPI_CFG2_CPHA, cphaType);
	setOneBitRegister(pSPI_Handler->pSPI.pCFG2, SPI_CFG2_MASTER, spiMode);

	setTwoBitRegister(pSPI_Handler->pSPI.pCFG2, SPI_CFG2_COMM, commType);
	//TODO set baudRate;
}


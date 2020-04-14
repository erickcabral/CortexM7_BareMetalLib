/*
 * spiDriver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Erick Cabral
 */

#ifndef SPIDRIVER_H_
#define SPIDRIVER_H_

#include "gpioDriver.h"

/**
 * @SPI PERIPHERALS BASEADDRESSES
 */

#define SPI1					1
#define SPI1_BASEADDRESS		0x40013000
#define SPI2					2
#define SPI2_BASEADDRESS		0x40003800
#define SPI3					3
#define SPI3_BASEADDRESS		0x40003C00
#define SPI4					4
#define SPI4_BASEADDRESS		0x40013400
#define SPI5					5
#define SPI5_BASEADDRESS		0x40015000
#define SPI6					6
#define SPI6_BASEADDRESS		0x58001400

/**
 * @SPI2S_CR1 REGISGTERS BITS
 */
#define SPI2S_CR1_IOLOCK		16
#define SPI2S_CR1_TCRCINI		15
#define SPI2S_CR1_RCRCINI		14
#define SPI2S_CR1_CRC33_17		13
#define SPI2S_CR1_SSI			12
#define SPI2S_CR1_HDDIR			11
#define SPI2S_CR1_CSUSP			10
#define SPI2S_CR1_CSTART		9
#define SPI2S_CR1_MASRX			8
#define SPI2S_CR1_SPE			0

/**
 * @SPI2S_IER REGISGTERS BITS
 */

#define SPI2S_IER_TSERFIE 	10
#define SPI2S_IER_MODFIE	9
#define SPI2S_IER_TIFREIE	8
#define SPI2S_IER_CRCEIE	7
#define SPI2S_IER_OVRIE		6
#define SPI2S_IER_UDRIE		5
#define SPI2S_IER_TXTFIE	4
#define SPI2S_IER_EOTIE		3
#define SPI2S_IER_DXPIE		2
#define SPI2S_IER_TXPIE		1
#define SPI2S_IER_RXPIE		0

/**
 * @SPI_I2SCFGR REGISGTERS BITS
 */
#define SPI_I2SCFGR_MCKOE		26
#define SPI_I2SCFGR_ODD			24
#define SPI_I2SCFGR_I2SDIV		16
#define SPI_I2SCFGR_DATFMT		14
#define SPI_I2SCFGR_WSINV		13
#define SPI_I2SCFGR_FIXCH		12
#define SPI_I2SCFGR_CKPOL		11
#define SPI_I2SCFGR_CHLEN		10
#define SPI_I2SCFGR_DATLEN		8
#define SPI_I2SCFGR_PCMSYNC		7
#define SPI_I2SCFGR_I2SSTD		4
#define SPI_I2SCFGR_I2SCFG		1
#define SPI_I2SCFGR_I2SMOD		0

/**
 * @SPI_CFG1 REGISGTERS BITS
 */
#define SPI_CFG1_MBR		28 	//[30:28]
#define SPI_CFG1_CRCEN		22
#define SPI_CFG1_CRCSIZE	16	//[20:16]
#define SPI_CFG1_TXDMAEN	15
#define SPI_CFG1_RXDMAEN	14
#define SPI_CFG1_UDRDET		11	//[12:11]
#define SPI_CFG1_UDRCFG		9	//[10:9]
#define SPI_CFG1_FTHLV		5	//[8:5]
#define SPI_CFG1_DSIZE		0	//[4:0]

/**
 * @SPI_CFG2 REGISGTERS BITS
 */
#define SPI_CFG2_AFCNTR		31
#define SPI_CFG2_SSOM		30
#define SPI_CFG2_SSOE		29
#define SPI_CFG2_SSIOP		28
#define SPI_CFG2_SSM		26
#define SPI_CFG2_CPOL		25
#define SPI_CFG2_CPHA		24
#define SPI_CFG2_LSBFRST	23
#define SPI_CFG2_MASTER		22
#define SPI_CFG2_SP			19	//[19:21]
#define SPI_CFG2_COMM		17	//[17:18]
#define SPI_CFG2_IOSWP		15
#define SPI_CFG2_MIDI		4	//[7:4]
#define SPI_CFG2_MSSI		0	//[3:0]


/**
 *  @COMM TYPES
 */
#define SPI_COMM_FULL_DUPLEX	0 	//0b00: full-duplex
#define SPI_COMM_SIMPLEX_TX		1 	//0b01: simplex transmitter
#define SPI_COMM_SIMPLEX_RX		2	//0b10: simplex receiver
#define SPI_COMM_HALF_DUPLEX	3	//0b11: salf-duplex
/**
 *  @SPI MODE TYPES
 */
#define SPI_MODE_SLAVE		0 	//
#define SPI_MODE_MASTER		1 	//
/**
 *  @CPOL MODE TYPES
 */
#define SPI_CPOL_LOW		0 	//
#define SPI_CPOL_HIGH		1 	//
/**
 *  @CPHA MODE TYPES
 */
#define SPI_CPHA_HIGH		1 	//
#define SPI_CPHA_LOW		0 	//

typedef struct {
	__volU32 *pCR1;		// OFFSET 0x00U
	__volU32 *pCR2;		// OFFSET 0x04U
	__volU32 *pCFG1;	// OFFSET 0x08U
	__volU32 *pCFG2;	// OFFSET 0x0CU
	__volU32 *pIER;		// OFFSET 0x10U
	__volU32 *pSR;		// OFFSET 0x14U
	__volU32 *pIFCR;	// OFFSET 0x18U
	__volU32 *pTXDR;	// OFFSET 0x20U
	__volU32 *pRXDR;	// OFFSET 0x30U
	__volU32 *pCRCPOLY;	// OFFSET 0x40U
	__volU32 *pTXCRC;	// OFFSET 0x44U
	__volU32 *pRXCRC;	// OFFSET 0x48U
	__volU32 *pUDRDR;	// OFFSET 0x4CU
	__volU32 *pI2SCFGR;	// OFFSET 0x50U
} SPI_Reg_t;

typedef struct {
	uint8_t spi;
	uint8_t spiMode;
	uint8_t spiCommType;
	uint8_t spiCPOL;
	uint8_t spiCPHA;
	uint8_t spiBaudRate;
} SPI_Config_t;

typedef struct {
	SPI_Reg_t pSPI;
	SPI_Config_t spiConfig;
	GPIOx_Confg_t pGPIOx;
} SPI_ConfigHandler_t;

void spi_initializeDriver(SPI_ConfigHandler_t *pSPI_Handler, uint8_t gpioX,
		uint8_t spiX);

void spi_enableSPIx(RCC_Reg_t *pRCC, uint8_t spiX);
uint32_t spi_GetSPIxBaseaddress(uint8_t spiX);
void spi_setupMasterPins(SPI_ConfigHandler_t *pSPI_Handler, uint8_t mosiPin,
		uint8_t misoPin, uint8_t sclkPin, uint8_t nssPin);
void spi_configSPI(SPI_ConfigHandler_t *pSPI_Handler, uint8_t spiMode,
		uint8_t commType, uint8_t cpolType, uint8_t cphaType, uint8_t baundRate);
/**
 *  @SPI CONFIGURATION STEPS
 *  BASIC CONFIG STEPS:
 *  1 - SELECT CORRECT GPIO AND ENABLE PINS -> MOSI_PIN, MISO_PIN, CLK_PIN, NSS_PIN
 *  2 - CONFIG MASTER or SLAVE MODE ->
 *  3 - CONFIG CPOL AND CPHA ->
 *  4 - CONFIG BAUD RATE ->
 *  5 - SET SPI ON
 *  --------------------------------------------------
 *  STEPS:
 *  1 - GPIOS ( MOSI_PIN, MISO_PIN, CLK_PIN, NSS_PIN )
 *  2 - Write to the SPI_CFG1 and SPI_CFG2 registers:
 *  (SSOM, SSOE, MBR[2:0], MIDI[3:0] and MSSI[3:0] are required at master mode only)
 *  (the MSSI bits take effect when SSOE is set)
 *  (MBR setting is required for slave at TI mode)
 *  (UDRDET[1:0] and UDRCFG[1:0] are required at slave mode only)
 *  (CRCSIZE[4:0] is required if CRCEN is set)
 *   //...
 *  3 - Write to the SPI_CR2 register to select length of the transfer, if it is not known TSIZE
 *  has to be programmed to zero.
 *
 */
#endif /* SPIDRIVER_H_ */

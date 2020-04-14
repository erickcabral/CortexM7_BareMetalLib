/*
 * mainSPI.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Erick Cabral
 */

#include "spiDriver.h"

int main(void){

	RCC_Reg_t rcc;
	initializeRCC(&rcc);
	enableGPIOx(&rcc, GPIOA, TRUE);
	spi_enableSPIx(&rcc, SPI1);

	SPI_ConfigHandler_t pSPI_Handler;
	spi_initializeDriver(&pSPI_Handler, GPIOA, SPI1);
	spi_setupMasterPins(&pSPI_Handler, PIN_0, PIN_1, PIN_2, PIN_3);
	spi_configSPI(&pSPI_Handler, SPI_MODE_MASTER, SPI_COMM_FULL_DUPLEX, SPI_CPOL_HIGH, SPI_CPHA_HIGH, 0);

	while(TRUE){

	}
}

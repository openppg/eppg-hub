/*
 * baro_LPS22HB.c
 *
 *  Created on: Feb 12, 2019
 *      Author: MiguelFAlvarez
 */

#include "main.h"
#include "baro_LPS22HB.h"
#include "spi.h"

uint8_t baroInit()
{
	if(baroReadReg(REG_WHOAMI) != WHO_AM_I_LPS22HB)
		return 0;

	//baroWriteReg()

	return 1;
}

void baroWriteReg(uint8_t regAddr, uint8_t data)
{

}

uint8_t baroReadReg(uint8_t regAddr)
{
	uint8_t dataTx[2];
	uint8_t dataRx[2];
	memset(dataTx, 0xFF, sizeof(dataTx));
	memset(dataRx, 0xFF, sizeof(dataRx));
	dataTx[0] = (regAddr & 0x7F) | 0x80;
	SPI_BARO_SELECT;
	SPIx_WriteReadData(dataTx, dataRx, sizeof(dataTx));
	SPI_BARO_DESELECT;
	return dataRx[1];
}

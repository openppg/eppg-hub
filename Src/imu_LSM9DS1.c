/*
 * imu_LSM9DS1.c
 *
 *  Created on: Feb 12, 2019
 *      Author: MiguelFAlvarez
 */

#include "main.h"
#include "imu_LSM9DS1.h"
#include "spi.h"

uint8_t imuInit()
{
	if(imuReadReg(REG_IMU_WHOAMI) != WHO_AM_I_LSM9DS1_IMU)
		return 0;

	return 1;
}

void imuWriteReg(uint8_t regAddr, uint8_t data)
{

}

uint8_t imuReadReg(uint8_t regAddr)
{
	uint8_t dataTx[2];
	uint8_t dataRx[2];
	memset(dataTx, 0xFF, sizeof(dataTx));
	memset(dataRx, 0xFF, sizeof(dataRx));
	dataTx[0] = (regAddr & 0x7F) | 0x80;
	SPI_IMU_SELECT;
	SPIx_WriteReadData(dataTx, dataRx, sizeof(dataTx));
	SPI_IMU_DESELECT;
	return dataRx[1];
}


uint8_t magInit()
{
	if(magReadReg(REG_MAG_WHOAMI) != WHO_AM_I_LSM9DS1_MAG)
		return 0;

	return 1;
}

void magWriteReg(uint8_t regAddr, uint8_t data)
{

}

uint8_t magReadReg(uint8_t regAddr)
{
	uint8_t dataTx[2];
	uint8_t dataRx[2];
	memset(dataTx, 0xFF, sizeof(dataTx));
	memset(dataRx, 0xFF, sizeof(dataRx));
	dataTx[0] = (regAddr & 0x7F) | 0x80;
	SPI_MAG_SELECT;
	SPIx_WriteReadData(dataTx, dataRx, sizeof(dataTx));
	SPI_MAG_DESELECT;
	return dataRx[1];
}

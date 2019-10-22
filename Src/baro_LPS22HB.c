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

void baroReadPressTemp(float *pressure, float *temperature)
{
	baroWriteReg(REG_CTRL_REG2,0x01);// start one shot conversion

	HAL_Delay(15);// wait 15 ms

	int status=baroReadReg(REG_STATUS);

	while(!((status&ST_TDA)&&(status&ST_PDA))) // wait for data ready flag
	{
		HAL_Delay(5); // delay more times
		status=baroReadReg(REG_STATUS);
	}

	// read back all value registers( total 5)

	uint8_t dataRx[6];

	for(int i=0;i<5;i++)
	{
		dataRx[i+1]=baroReadReg(i+REG_PRESS_OUT_XL);
	}


	int32_t unscaled_pressure=dataRx[1]|(dataRx[2]<<8)|(dataRx[3]<<16);

	/* convert the 2's complement 24 bit to 2's complement 32 bit */
	if(unscaled_pressure & 0x00800000)
	{
		unscaled_pressure |= 0xFF000000;
	}

	*pressure=(float)unscaled_pressure*100.0f/4096.0f;

	int16_t unscaled_temperature=dataRx[4]|(dataRx[5]<<8);
	*temperature=(float)unscaled_temperature/100.0f;
}

void baroWriteReg(uint8_t regAddr, uint8_t data)
{
	uint8_t dataTx[2];

	dataTx[0] = (regAddr & 0x7F);
	dataTx[1] = data;
	SPI_BARO_SELECT;
	SPIx_WriteData(dataTx, sizeof(dataTx));
	SPI_BARO_DESELECT;

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

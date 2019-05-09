/*
 * baro_LPS22HB.h
 *
 *  Created on: Feb 12, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef BARO_LPS22HB_H_
#define BARO_LPS22HB_H_

#include <stdint.h>

#define WHO_AM_I_LPS22HB	0xB1

typedef enum
{
	REG_WHOAMI = 0x0F,

}BARO_REGS;

uint8_t baroInit();
void baroWriteReg(uint8_t regAddr, uint8_t data);
uint8_t baroReadReg(uint8_t regAddr);

#endif /* BARO_LPS22HB_H_ */

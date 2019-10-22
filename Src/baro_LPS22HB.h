/*
 * baro_LPS22HB.h
 *
 */

#ifndef BARO_LPS22HB_H_
#define BARO_LPS22HB_H_

#include <stdint.h>

#define WHO_AM_I_LPS22HB	0xB1

typedef enum
{
	REG_WHOAMI = 0x0F,

	REG_CTRL_REG1=0x10,
	REG_CTRL_REG2=0x11,
	REG_CTRL_REG3=0x12,

	REG_STATUS=0x27,

	REG_PRESS_OUT_XL=0x28,
	REG_PRESS_OUT_L=0x29,
	REG_PRESS_OUT_H=0x2A,

	REG_TEMP_OUT_L=0x2B,
	REG_TEMP_OUT_H=0x2C,


}BARO_REGS;


typedef enum
{
	ST_TOR=0x20,
	ST_POR=0x10,

	ST_TDA=0x02,
	ST_PDA=0x01,
}BARO_STATUS;

uint8_t baroInit();
void baroWriteReg(uint8_t regAddr, uint8_t data);
uint8_t baroReadReg(uint8_t regAddr);
void baroReadPressTemp(uint32_t *pressure, int16_t *temperature);

#endif /* BARO_LPS22HB_H_ */

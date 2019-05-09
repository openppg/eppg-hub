/*
 * imu_LSM9DS1.h
 *
 *  Created on: Feb 12, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef IMU_LSM9DS1_H_
#define IMU_LSM9DS1_H_

#include <stdint.h>

#define WHO_AM_I_LSM9DS1_IMU	0x68
#define WHO_AM_I_LSM9DS1_MAG	0x3D

typedef enum
{
	REG_IMU_WHOAMI = 0x0F,

}IMU_REGS;

typedef enum
{
	REG_MAG_WHOAMI = 0x0F,

}MAG_REGS;

uint8_t imuInit();
void imuWriteReg(uint8_t regAddr, uint8_t data);
uint8_t imuReadReg(uint8_t regAddr);

uint8_t magInit();
void magWriteReg(uint8_t regAddr, uint8_t data);
uint8_t magReadReg(uint8_t regAddr);

#endif /* IMU_LSM9DS1_H_ */

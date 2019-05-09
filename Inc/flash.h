/*
 * flash.h
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef FLASH_H_
#define FLASH_H_

HAL_StatusTypeDef flashInit();
HAL_StatusTypeDef flashWriteParams();
HAL_StatusTypeDef flashEraseParams();
#endif /* FLASH_H_ */

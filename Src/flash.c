/*
 * flash.c
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "flash.h"

HAL_StatusTypeDef flashInit()
{
	HAL_StatusTypeDef status = HAL_OK;
	memcpy(&parameters, &paramSpace, sizeof(paramSpace));
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR );

	if(parameters.version != VERSION || parameters.candidate != CANDIDATE || parameters.build != BUILD)
	{
		flashEraseParams();

		memset(&parameters, 0, sizeof(parameters));
		parameters.version = VERSION;
		parameters.candidate = CANDIDATE;
		parameters.build = BUILD;

		status = flashWriteParams();
	}

	HAL_FLASH_Lock();
	return status;
}


HAL_StatusTypeDef flashWriteParams()
{
	HAL_StatusTypeDef status = HAL_OK;
	flashEraseParams();
	HAL_FLASH_Unlock();
	for(uint32_t i = 0; i<sizeof(paramSpace); i+=4)
	{
	uint32_t* valPtr = (uint32_t*)&parameters.raw[i];
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&paramSpace.raw[i], (uint32_t)(*valPtr));
	if(status != HAL_OK)
		Error_Handler();
	}

	HAL_FLASH_Lock();
	return status;
}


HAL_StatusTypeDef flashEraseParams()
{
	HAL_StatusTypeDef status = HAL_OK;
	HAL_FLASH_Unlock();

	uint32_t errorPtr;
	FLASH_EraseInitTypeDef eraseInit;
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.PageAddress = (uint32_t)&paramSpace;
	eraseInit.NbPages = 1;
	status = HAL_FLASHEx_Erase(&eraseInit, &errorPtr);

	HAL_FLASH_Lock();

	return status;
}




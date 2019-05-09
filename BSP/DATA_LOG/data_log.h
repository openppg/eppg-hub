/*
 * data_log.h
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef DATA_LOG_H_
#define DATA_LOG_H_

#include "fatfs.h"

FRESULT logInit(TCHAR* fileName);
FRESULT logDeInit();
FRESULT logOpenFile(TCHAR* fileName);
FRESULT logCloseFile();
FRESULT logWriteData(uint8_t* data, uint32_t bytesToWrite, uint32_t bytesWritten);
FRESULT formatCard();

#endif /* DATA_LOG_H_ */

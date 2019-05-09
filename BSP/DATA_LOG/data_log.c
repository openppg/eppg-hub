/*
 * data_log.c
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */

#include "data_log.h"
#include <string.h>

FRESULT logInit(TCHAR* fileName)
{
	FRESULT ret = 0;
	/*##-1- Link the micro SD disk I/O driver ##################################*/
	if(retUSER == 0)
	{
		/*##-2- Register the file system object to the FatFs module ##############*/
		if((ret = f_mount(&USERFatFS, (TCHAR const*)USERPath, 0)) != FR_OK)
		{
		   return ret;
		}
		else
		{
			TCHAR fileName2[100];
			memset(fileName2, 0, sizeof(fileName2));
			uint32_t fileNumber = 0;
			sprintf(fileName2, "%s%u.csv", fileName, fileNumber++);
			ret = f_open(&USERFile, fileName2, FA_CREATE_NEW | FA_WRITE);
			while(ret == FR_EXIST)
			{
				sprintf(fileName2, "%s%u.csv", fileName, fileNumber++);
				/*##-4- Create and Open a new text file object with write access #####*/
				ret = f_open(&USERFile, fileName2, FA_CREATE_NEW | FA_WRITE);
			}
			strcpy(fileName, fileName2);

			//We're using f_sync() now instead of reopening and closing the file for every write.
			//if(ret == FR_OK)
			//	ret = f_close(&USERFile);

			return ret;
		}
	}
	else
		return retUSER;
}

FRESULT logDeInit()
{
	FRESULT ret = 0;
	/*##-6- Close the open text file #################################*/
	ret = f_close(&USERFile);
	return ret;
	/*##-11- Unlink the RAM disk I/O driver ####################################*/
	//FATFS_UnLinkDriver(USERPath);
}

FRESULT logOpenFile(TCHAR* fileName)
{
	FRESULT ret = 0;
	if((ret = f_open(&USERFile, fileName, FA_OPEN_EXISTING | FA_WRITE)) != FR_OK)
		goto writeEnd;

writeEnd:
	return ret;
}

FRESULT logCloseFile()
{
	FRESULT ret = 0;
	if((ret = f_close(&USERFile)) != FR_OK)
		goto writeEnd;

writeEnd:
	return ret;
}

FRESULT logWriteData(uint8_t* data, uint32_t bytesToWrite, uint32_t bytesWritten)
{
	FRESULT ret = 0;
	/*##-5- Write data to the text file ################################*/
	if((ret = f_lseek(&USERFile, f_size(&USERFile))) != FR_OK)
		goto writeEnd;
	if((ret = f_write(&USERFile, data, bytesToWrite, (void *)&bytesWritten)) != FR_OK)
		goto writeEnd;
	if((ret = f_sync(&USERFile)) != FR_OK)
		goto writeEnd;

writeEnd:
	return ret;
}

FRESULT formatCard()
{
	/*##-3- Create a FAT file system (format) on the logical drive #########*/
	/* WARNING: Formatting the uSD card will delete all content on the device */
	return f_mkfs((TCHAR const*)USERPath, 0, 0);
}

/*
 * escTelemetry.c
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */


#include "escTelemetry.h"

STR_ESC_DATA escData[NUM_ESCS];

UN_ESC_PACKET escPacket[NUM_ESCS];

void escInit()
{
	memset(escData, 0xFF, sizeof(escData));
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		escData[i].timeStamp = 0;
		escData[i].packetNum = 0;
	}
	// Future esc's might require some sort of telemetry initialization data exchange
}

ESC_STATUS escParseData(uint8_t *data, uint8_t size, uint8_t escIndex)
{
	//check for allowable packet size and esc indexbefore continuing
	//if(size > ESC_HW_XROTOR_PACKET_LENGTH || escIndex > NUM_ESCS)
	//	return ESC_STATUS_INVALID_DATA;

	uint32_t currentTime = HAL_GetTick();
	static uint32_t lastTime[NUM_ESCS];
	static uint32_t mahRaw[NUM_ESCS];

	if(escPacket[escIndex].escHwXrotor.header == ESC_HW_XROTOR_START_BYTE && escPacket[escIndex].escHwXrotor.packetLength == ESC_HW_XROTOR_PACKET_LENGTH && escPacket[escIndex].escHwXrotor.firmwareVersion == ESC_HW_XROTOR_FIRMWARE)
	{
		uint16_t checksum = 0;
		for(uint8_t i = 0; i<ESC_HW_XROTOR_PACKET_LENGTH; i++)
		{
			checksum += escPacket[escIndex].raw[i];
		}

		if(checksum != escPacket[escIndex].escHwXrotor.checksum)
			return ESC_STATUS_BAD_CHECKSUM;

		escData[escIndex].timeStamp = currentTime;
		escData[escIndex].packetNum = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.packetNum);
		escData[escIndex].throttleInput = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.throttleVal)*100/1024;
		escData[escIndex].throttleOutput = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.actualPwmOut)*100/1024;
		escData[escIndex].rpm = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.actualRpm)*600/PARAM_MOTOR_POLES;	//TODO Need to verify pole count is correct. Initial formula of xxxx*10/pole pairs/gear ratio seemed wrong. Multiplied by 60 and now it seems correct. Initial forumla must've been rps instead of rpm.
		escData[escIndex].voltage = 1000*UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.voltageRaw)*11/650;
		escData[escIndex].current = 1000*UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.currentRaw)*33/2020;
		mahRaw[escIndex] += escData[escIndex].current*(currentTime-lastTime[escIndex]);
		escData[escIndex].mah = mahRaw[escIndex]/HRS_TO_MS;	//TODO Might need to increase unit size to avoid resolution loss
		escData[escIndex].capTemp = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.capTemp);	//TODO need to figure out resistance of NTC sensor
		escData[escIndex].fetTemp = UINT16_ENDIAN(escPacket[escIndex].escHwXrotor.fetTemp);

		lastTime[escIndex] = currentTime;
		return ESC_STATUS_GOOD;
	}
	/*
	else if()	//More esc types?
	{

	}
	*/
	else
	{
		escData[escIndex].throttleInput = 0xFF;
		escData[escIndex].throttleOutput = 0xFF;
		escData[escIndex].rpm = 0xFFFF;
		escData[escIndex].voltage = 0xFFFFFFFF;
		escData[escIndex].current = 0xFFFFFFFF;
		//escData[escIndex].mah		//Dont want to reset this
		escData[escIndex].capTemp = 0xFF;
		escData[escIndex].fetTemp = 0xFF;
		return ESC_STATUS_INVALID_DATA;
	}
}

uint32_t escGetAvgVoltage()
{
	uint32_t avgVoltage = 0;
	uint8_t validValues = 0;
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		if(escData[i].voltage != 0xFFFFFFFF)
		{
			validValues++;
			avgVoltage += escData[i].voltage;
		}
	}
	avgVoltage = avgVoltage/validValues;
	return avgVoltage;
}

uint32_t escGetTotalMah()
{
	uint32_t totalMah = 0;

	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		totalMah += escData[i].mah;
	}
	return totalMah;
}

uint32_t escGetTotalCurrent()
{
	static uint32_t avgCurrent = 0;
	uint32_t totalCurrent = 0;
	uint8_t validValues = 0;
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		if(escData[i].current != 0xFFFFFFFF)
		{
			validValues++;
			totalCurrent += escData[i].current;
		}
	}
	avgCurrent = ema_u32(totalCurrent, avgCurrent, EMA_CURRENT_ALPHA);
	return avgCurrent;
}

uint16_t escGetAvgRpm()
{
	static uint16_t avgRpmEma = 0;
	uint16_t avgRpm = 0;
	uint8_t validValues = 0;
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		if(escData[i].rpm != 0xFFFF)
		{
			validValues++;
			avgRpm += escData[i].rpm;
		}
	}
	avgRpm = avgRpm/validValues;
	avgRpmEma = ema_u16(avgRpm, avgRpmEma, EMA_RPM_ALPHA);
	return avgRpmEma;
}

uint8_t escGetAvgCapTemp()
{
	uint8_t avgTemp = 0;
	uint8_t validValues = 0;
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		if(escData[i].capTemp != 0xFFFFFFFF)
		{
			validValues++;
			avgTemp += escData[i].capTemp;
		}
	}
	avgTemp = avgTemp/validValues;
	return avgTemp;
}

uint8_t escGetAvgFetTemp()
{
	uint8_t avgTemp = 0;
	uint8_t validValues = 0;
	for(uint8_t i = 0; i<NUM_ESCS; i++)
	{
		if(escData[i].fetTemp != 0xFFFFFFFF)
		{
			validValues++;
			avgTemp += escData[i].fetTemp;
		}
	}
	avgTemp = avgTemp/validValues;
	return avgTemp;
}


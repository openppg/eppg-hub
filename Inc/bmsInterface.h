/*
 * bmsInterface.h
 *
 *  Created on: Mar 31, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef BMSINTERFACE_H_
#define BMSINTERFACE_H_

#include <stdint.h>

#define BMS_VER 0x00
#define BMS2HUB_ID	0x50
#define HUB2BMS_ID 0x60

#pragma pack(push, 1)

typedef struct
{
	uint16_t cellVoltage[7];
}STR_BATT_DATA;

typedef struct
{
	uint8_t version;
	uint8_t id;
	uint8_t length;
	STR_BATT_DATA battData[8];
	uint16_t crc;
}STR_BMS2HUB_MSG;

typedef struct
{
	uint8_t version;
	uint8_t id;
	uint8_t length;
	uint8_t rfu;
	uint16_t crc;
}STR_HUB2BMS_ACK;

#pragma pack(pop)

void receiveBmsData(uint8_t *buf, uint32_t size);
void sendBmsAck();


#endif /* BMSINTERFACE_H_ */

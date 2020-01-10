/*
 * controllerInterface.h
 *
 *  Created on: Mar 3, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef CONTROLLERINTERFACE_H_
#define CONTROLLERINTERFACE_H_

#include <stdint.h>

//#define INTERFACE_CONTROLLER
#define INTERFACE_HUB
#define CTRL_VER 0x01
#define CTRL2HUB_ID	0x10
#define HUB2CTRL_ID 0x20

#pragma pack(push, 1)
typedef struct
{
	uint8_t version;
	uint8_t id;
	uint8_t length;
	uint8_t armed;
	uint16_t throttlePercent; //0 to 1000
	uint16_t crc;
}STR_CTRL2HUB_MSG;

typedef struct
{
	uint8_t version;
	uint8_t id;
	uint8_t length;
	uint8_t armed;
	uint32_t voltage;
	uint32_t totalMah;
	uint32_t totalCurrent;
	uint16_t avgRpm;
	uint8_t avgCapTemp;
	uint8_t avgFetTemp;
	int16_t baroTemp; // degrees c
	uint32_t baroPressure; // hpa
	uint16_t crc;
}STR_HUB2CTRL_MSG;


#pragma pack(pop)
#ifdef INTERFACE_CONTROLLER
void sendControlData(uint8_t armed, uint16_t throttlePercent);
void receiveHubData(uint8_t *buf, uint32_t size);
#endif

#ifdef INTERFACE_HUB
void receiveControlData(uint8_t *buf, uint32_t size);
void sendHubData();
#endif

#endif /* CONTROLLERINTERFACE_H_ */

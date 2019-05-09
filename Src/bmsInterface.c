/*
 * bmsInterface.c
 *
 *  Created on: Mar 31, 2019
 *      Author: MiguelFAlvarez
 */

#include "bmsInterface.h"
#include "crc.h"
#include "usart.h"

static STR_BMS2HUB_MSG bmsData;
static STR_HUB2BMS_ACK bmsAck;

void receiveBmsData(uint8_t *buf, uint32_t size)
{
	if(size != sizeof(STR_BMS2HUB_MSG))
		return;

	memcpy((uint8_t*)&bmsData, buf, sizeof(STR_BMS2HUB_MSG));
	uint16_t crc = crc16((uint8_t*)&bmsData, sizeof(STR_BMS2HUB_MSG) - 2);

	if(crc != bmsData.crc)
		return;


	sendBmsAck();
}

void sendBmsAck()
{
	memset((uint8_t*)&bmsAck, 0, sizeof(STR_HUB2BMS_ACK));
	bmsAck.version = BMS_VER;
	bmsAck.id = HUB2BMS_ID;
	bmsAck.length = sizeof(STR_HUB2BMS_ACK);

	bmsAck.crc = crc16((uint8_t*)&bmsAck, sizeof(STR_HUB2BMS_ACK) - 2);

	sendToBms((uint8_t*)&bmsAck, sizeof(STR_HUB2BMS_ACK));
}

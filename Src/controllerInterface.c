/*
 * controllerInterface.c
 *
 *  Created on: Mar 3, 2019
 *      Author: MiguelFAlvarez
 */

#include "controllerInterface.h"
#include "crc.h"
#include "usart.h"
#include "tim.h"
#include "escTelemetry.h"

#include "baro_proc.h"

static STR_CTRL2HUB_MSG controlData;
static STR_HUB2CTRL_MSG hubData;
static uint8_t armed = 0;

#ifdef INTERFACE_CONTROLLER
void sendControlData(uint8_t armed, uint16_t throttlePercent)
{
	memset((uint8_t*)&controlData, 0, sizeof(STR_CTRL2HUB_MSG));
	controlData.version = CTRL_VER;
	controlData.id = CTRL2HUB_ID;
	controlData.length = sizeof(STR_CTRL2HUB_MSG);
	controlData.armed = armed;
	controlData.throttlePercent = throttlePercent;
	controlData.crc = crc16((uint8_t*)&controlData, sizeof(STR_CTRL2HUB_MSG) - 2);
	UART_RS485_BeginTransmit(HUB_UART, HUB_TX_DMA, HUB_TX_DMA_CH, (uint8_t*)&controlData, sizeof(STR_CTRL2HUB_MSG));
}

void receiveHubData(uint8_t *buf, uint32_t size)
{

}
#endif

#ifdef INTERFACE_HUB
void receiveControlData(uint8_t *buf, uint32_t size)
{
	if(size != sizeof(STR_CTRL2HUB_MSG))
		return;

	memcpy((uint8_t*)&controlData, buf, sizeof(STR_CTRL2HUB_MSG));
	uint16_t crc = crc16((uint8_t*)&controlData, sizeof(STR_CTRL2HUB_MSG) - 2);

	if(crc != controlData.crc)
		return;

	if(controlData.armed)
	{
		armed = 1;
		sendThrottlePulse(controlData.throttlePercent, 1);
		sendThrottlePulse(controlData.throttlePercent, 2);
		sendThrottlePulse(controlData.throttlePercent, 3);
		sendThrottlePulse(controlData.throttlePercent, 4);
	}
	else
		armed = 0;

	sendHubData();
}

void sendHubData()
{
	memset((uint8_t*)&hubData, 0, sizeof(STR_HUB2CTRL_MSG));
	hubData.version = CTRL_VER;
	hubData.id = HUB2CTRL_ID;
	hubData.length = sizeof(STR_HUB2CTRL_MSG);
	hubData.armed = armed;
	hubData.voltage = escGetAvgVoltage();
	hubData.totalMah = escGetTotalMah();
	hubData.totalCurrent = escGetTotalCurrent();
	hubData.avgRpm = escGetAvgRpm();
	hubData.avgCapTemp = escGetAvgCapTemp();
	hubData.avgFetTemp = escGetAvgFetTemp();

	hubData.baroTemp = baroGetTempAvg();
	hubData.baroPressure = baroGetAvg();

	hubData.crc = crc16((uint8_t*)&hubData, sizeof(STR_HUB2CTRL_MSG) - 2);

	sendToController((uint8_t*)&hubData, sizeof(STR_HUB2CTRL_MSG));
}

#endif

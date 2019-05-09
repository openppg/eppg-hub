/*
 * escTelemetry.h
 *
 *  Created on: Feb 5, 2019
 *      Author: MiguelFAlvarez
 */

#ifndef ESCTELEMETRY_H_
#define ESCTELEMETRY_H_

#include <string.h>
#include <stdint.h>

#define UINT16_ENDIAN(a) ((a>>8 & 0x00FF)|(a<<8 & 0xFF00))
//ESC list enum to use as an index for things such as conversion tables (e.g. converting raw current draw or voltage to a value with proper units like amps or volts)
typedef enum
{
	ESC_HW_XROTOR,
}ESC_TYPE;

#define NUM_ESCS				4
#define ESC_MAX_PACKET_SIZE		100		//largest possible packet size for esc

#define HRS_TO_MS				3600000
//TODO need to make these a part of flash parameters instead
#define PARAM_ESC_HARDWARE		ESC_HW_XROTOR
#define PARAM_MOTOR_POLES		36

#pragma pack(push, 1)

//Any unused values should be filled with 0xFF's
typedef struct
{
	uint32_t timeStamp;
	uint32_t packetNum;
	uint8_t throttleInput;	// % 0 to 100
	uint8_t throttleOutput;// % 0 to 100
	uint16_t rpm;			// rpm
	uint32_t voltage;		// mV
	uint32_t current;		// mA
	uint32_t mah;			// mAh
	uint8_t capTemp;		// C
	uint8_t fetTemp;		// C

}STR_ESC_DATA;

extern STR_ESC_DATA escData[NUM_ESCS];



//Hobbywing XRotor esc. Current esc used for OpenPPG
#define ESC_HW_XROTOR_START_BYTE	0x9B
#define ESC_HW_XROTOR_PACKET_LENGTH	22	//doesnt include checksum
#define ESC_HW_XROTOR_FIRMWARE		3

typedef struct
{
	uint8_t header;				// 0x9B
	uint8_t packetLength;		// 22
	uint8_t firmwareVersion;	// 3
	uint8_t rfu1;
	uint16_t packetNum;
	uint16_t throttleVal;		// Range from 0-1024
	uint16_t actualPwmOut;		// throttle % currently being applied to escs
	uint16_t actualRpm;			// The actual electric rev of a motor. The actual speed = xxxx*10/pole pairs/gear ratio. Eg, if the HEX value is 0xC350, the motor is 14-pole (7 pairs), the gear ration is 1:1, then the actual rpm of the motor is 50000*10/7/1=71428
	uint16_t voltageRaw;		// voltage = voltageRaw*11/650 V
	uint16_t currentRaw;		// current = currentRaw*33/2020 A
	uint8_t rfu2[2];
	uint16_t capTemp;			//
	uint16_t fetTemp;
	uint16_t checksum;			// sum of all bytes from 0 to 21
}STR_ESC_HW_XROTOR;


typedef union
{
	STR_ESC_HW_XROTOR escHwXrotor;

	uint8_t raw[ESC_MAX_PACKET_SIZE];	//currently only one esc type with a packet length of 24 bytes
}UN_ESC_PACKET;

extern UN_ESC_PACKET escPacket[NUM_ESCS];

#pragma pack(pop)

typedef enum
{
	ESC_STATUS_GOOD,
	ESC_STATUS_INVALID_DATA,
	ESC_STATUS_BAD_CHECKSUM

}ESC_STATUS;

void escInit();
uint8_t escParseData(uint8_t *data, uint8_t size, uint8_t escIndex);

uint32_t escGetAvgVoltage();
uint32_t escGetTotalMah();
uint32_t escGetTotalCurrent();
uint16_t escGetAvgRpm();
uint8_t escGetAvgCapTemp();
uint8_t escGetAvgFetTemp();
#endif /* ESCTELEMETRY_H_ */

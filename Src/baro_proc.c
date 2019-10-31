#include "main.h"
#include "baro_LPS22HB.h"

#define BARO_AVG_COUNT 4

static uint32_t baro_value[BARO_AVG_COUNT];
static uint32_t baro_temp_value[BARO_AVG_COUNT];
static int32_t baro_value_idx;


void baroAvgInit()
{
	int16_t init_temp_value;
	uint32_t init_value= baroReadPressTemp(&init_temp_value);

	// set all buf to the inital value
	for(int i=0;i<BARO_AVG_COUNT;i++)
	{
		baro_value[i]=init_value;
		baro_temp_value[i]=init_temp_value;
	}

	baro_value_idx=0;
}

void baroSample()
{
	int16_t baro_temp_reading;
	uint32_t baro_reading= baroReadPressTemp(&baro_temp_reading);

	if(baro_reading!=0xffffffff)
	{// save the value if it is valid
		baro_value[baro_value_idx]=baro_reading;
		baro_temp_value[baro_value_idx]=baro_temp_reading;
		baro_value_idx++;

		if(baro_value_idx>=BARO_AVG_COUNT)
		{
			baro_value_idx=0;
		}
	}

}

uint32_t baroGetAvg()
{
	uint32_t total=0;

	for(int i=0;i<BARO_AVG_COUNT;i++)
	{
		total+=baro_value[i];
	}

	return total/BARO_AVG_COUNT;
}

int16_t baroGetTempAvg()
{
	int32_t total=0;

	for(int i=0;i<BARO_AVG_COUNT;i++)
	{
		total+=baro_temp_value[i];
	}

	return total/BARO_AVG_COUNT;
}

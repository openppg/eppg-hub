#include "main.h"
#include "baro_LPS22HB.h"

#define BARO_AVG_COUNT 4

static uint32_t baro_value[BARO_AVG_COUNT];
static int32_t baro_value_idx;


void baroAvgInit()
{
	uint32_t init_value= baroReadPressTemp();

	// set all buf to the inital value
	for(int i=0;i<BARO_AVG_COUNT;i++)
	{
		baro_value[i]=init_value;
	}

	baro_value_idx=0;
}

void baroSample()
{
	uint32_t baro_reading= baroReadPressTemp();

	if(baro_reading!=0xffffffff)
	{// save the value if it is valid
		baro_value[baro_value_idx]=baro_reading;
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

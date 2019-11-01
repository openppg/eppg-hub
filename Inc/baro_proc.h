/*
 * baro_proc.h
 *
 *  Created on: Oct 23, 2019
 */

#ifndef BARO_PROC_H_
#define BARO_PROC_H_

void baroAvgInit();
void baroSample();
uint32_t baroGetAvg();
int16_t baroGetTempAvg();

#endif /* BARO_PROC_H_ */

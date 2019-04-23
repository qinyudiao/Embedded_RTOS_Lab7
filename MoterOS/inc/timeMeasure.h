#ifndef __MEASURE_TIME_H
#define __MEASURE_TIME_H


void timeMeasureInit(void);  // initialize time measurement. At main, it should be exist
	
void timeMeasurestart(void);


void disableTimeget(void);

void enableTimeget(void);

int getDisablePercent(void);
#endif
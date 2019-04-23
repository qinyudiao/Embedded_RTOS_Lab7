
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "tm4c123gh6pm.h"
#include "OS.h"
#include "timeMeasure.h"

static unsigned long long __startTime;
static unsigned long long __total_interupt_time;
static unsigned long long __disable_interupt_T;
static bool enabled;
static bool start ;
static bool intr_stack[64];
static int stackidx = 0;

void timeMeasureInit(void)
{
	enabled = false;
	start = false;
	__startTime = OS_Time();
	__total_interupt_time = 0;
} // initialize time measurement. At main, it should be exist
	
void timeMeasurestart(void)
{
	start = true;
	enabled =true;
}


void disableTimeget(void)
{
	if(start == true){
		intr_stack[stackidx++] = enabled;
		enabled = false;
		__disable_interupt_T = OS_Time();
	}
}

void enableTimeget(void)
{
	
	if (start == true && stackidx>0){
		enabled = intr_stack[--stackidx];
	}
	if(start == true && enabled== true){
		unsigned long long cur = OS_Time();
		unsigned long long diff = OS_TimeDifference(__disable_interupt_T,cur);
		__total_interupt_time += diff;
	}
}

int getDisablePercent(void)
{
	if (start ){
	unsigned long long cur = OS_Time();
	unsigned long long diff = OS_TimeDifference(__startTime,cur);

	return (int)(__total_interupt_time / (diff/100));
	}
	else 
		return 0;
	
}


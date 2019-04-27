#include "IR.h"
#include "ADC.h"
#include "OS.h"
#include <stdio.h>
#include "LED.h"

static long dis;

int channelData[4];

#define OS_FIFO_SIZE 16
volatile uint32_t *getPt;
volatile uint32_t *putPt;
static uint32_t fifo[OS_FIFO_SIZE];
static Sema4Type ff_DataNum;
//static Sema4Type ff_mutex;		// mutex not needed, because only one consumer

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void Fifo_Init() {
	putPt = getPt = &fifo[0];
	OS_InitSemaphore(&ff_DataNum, 0);
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers
//  this function can not disable or enable interrupts
int Fifo_Put(unsigned long data) {
	// only one background producer thread, no critical section
	/*
	 * Notice here, cannot just use semaphore.value to check whether it's full or not
	 * because in Get, DataNum may be decremented, then it's switched out or interrupted by Put
	 * before the data is actually recorded. Thus we need to make sure FIFO is never full
	 */
	uint32_t volatile *nextPutPt;
	nextPutPt = putPt+1;
	if(nextPutPt == &fifo[OS_FIFO_SIZE]){
		nextPutPt = &fifo[0];  // wrap
	}
	if(nextPutPt == getPt){
		return 0;      // Failed, fifo full; Since cannot wait here (in ISR)
	}
	*putPt = data;
	putPt = nextPutPt;
	OS_Signal(&ff_DataNum);		// increment current data number
	return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data
unsigned long Fifo_Get(void) {
	OS_Wait(&ff_DataNum);	    // decrement current data number; if empty, spin or block
								// wait until other consumer thread finishes, then lock
	unsigned long data = *getPt;
	long sr = StartCritical();     // make the getPt increment process atomic
	getPt++;
	if (getPt == &fifo[OS_FIFO_SIZE])
		getPt = &fifo[0];
	EndCritical(sr);
	return data;
}
// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long Fifo_Size(void) {
	if(putPt < getPt){
		return ((unsigned short)(putPt-getPt+(OS_FIFO_SIZE*sizeof(uint32_t)))/sizeof(uint32_t));
	}
	return ((unsigned short)(putPt-getPt)/sizeof(uint32_t));
}

//------------ADC2millimeter------------
// convert 12-bit ADC to distance in 1mm
// it is known the expected range is 100 to 800 mm
// Input:  adcSample 0 to 4095
// Output: distance in 1mm
static long ADC2millimeter(unsigned long adcSample)
{
  if (adcSample < 494)
    return 799; // maximum distance 80cm
  return (268130 / (adcSample - 159));

// 24720 / x + 322
}

static long median(long u1, long u2, long u3)
{
  long result;
  if (u1 > u2)
    if (u2 > u3)
      result = u2; // u1>u2,u2>u3       u1>u2>u3
    else if (u1 > u3)
      result = u3; // u1>u2,u3>u2,u1>u3 u1>u3>u2
    else
      result = u1; // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else if (u3 > u2)
    result = u2; // u2>u1,u3>u2       u3>u2>u1
  else if (u1 > u3)
    result = u1; // u2>u1,u2>u3,u1>u3 u2>u1>u3
  else
    result = u3; // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return (result);
}

static int len = 0;
void ADC_test() {
	int run = 0;
	while (run < 40) {
// if (Fifo_Size() > 0) {
//	  LED_GREEN_TOGGLE();
//	 	printf("distance: %d mm\n\r", (int)Fifo_Get());
//	 run++;
// }
		IR_GetData(run % 4);
		run++;
		OS_Sleep(100);
 }
	OS_Kill();
}

long IR_getmm(void)
{
    long sr = StartCritical();
    long dis_copy = dis;
    EndCritical(sr);
    return dis_copy;
}


static void IR_handler(unsigned long data) {
	static long x1[4], x2[4], x3[4];
	  static int curSensorIndex = 0; 
    x3[curSensorIndex] = x2[curSensorIndex];
    x2[curSensorIndex] = x1[curSensorIndex];       // MACQ
    x1[curSensorIndex] = data; // channel set when calling ADC_Init
    long output = median(x1[curSensorIndex], x2[curSensorIndex], x3[curSensorIndex]); // 3-wide median filter
    dis = ADC2millimeter(output);
	  //Fifo_Put(dis);
		channelData[curSensorIndex++] = dis;
		if (curSensorIndex >= 4) curSensorIndex = 0;
}

/**
 * J5 on sensor board 
 * channel 0 (PE3)
 * frequency: 50 Hz
 */
void IR_Init(void) {
//    int res = ADC_Collect(0, 10, IR_handler);
			//LED_Init();
	    //Fifo_Init();
			uint32_t channels[4] = {0,1,2,3};
		  ADC_Collect4Chan(channels, 200, IR_handler);
	    // OS_AddThread(&ADC_test, 128, 4);
}

/*
 * @param index: the sensor index; J5 is 0 and J8 is 3
 * @return the latest measurement from that sensor, in unit of mm
 */
int IR_GetData(int index) {
	 // printf("index %d: distance %d mm\n\r", index, channelData[index]);
	 return channelData[index];
}

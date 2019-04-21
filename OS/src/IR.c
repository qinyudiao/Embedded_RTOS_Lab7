#include "IR.h"
#include "ADC.h"
#include "OS.h"
#include <stdio.h>
#include "LED.h"

static long dis;

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
 // if (len++ < 20)
	printf("distance: %d mm\n\r", (int)dis);
  OS_Kill();
}

static long x1, x2, x3;
static void IR_handler(unsigned long data) {
    x3 = x2;
    x2 = x1;       // MACQ
    x1 = data; // channel set when calling ADC_Init
    long output = median(x1, x2, x3); // 3-wide median filter
    dis = ADC2millimeter(data);
	  //dis = data;
    //OS_AddThread(&ADC_test, 128, 1);
}

/**
 * J5 on sensor board 
 * channel 0 (PE3)
 * frequency: 50 Hz
 */
void IR_Init(void) {
    int res = ADC_Collect(0, 10, IR_handler);
 }

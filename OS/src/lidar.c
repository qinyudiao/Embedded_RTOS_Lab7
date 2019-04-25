
#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "I2C.h"
#include "VL53L0X.h"
#include "UART.h"
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LED.h"
#include "xshut.h"

#define ADC_CLK_FREQ (80000000)
static int dis;
static Sema4Type serial;
int initLidar0 = 0;
int initLidar1 = 0;

int lidar_data[2];

int lidar_GetMeasurement(int index)
{
    VL53L0X_RangingMeasurementData_t measurement;
    VL53L0X_getSingleRangingMeasurement(&measurement, index);
    if (measurement.RangeStatus != 4)
    {
        return measurement.RangeMilliMeter;
    }
    else // out of range
        return -1;
}


void lidar_task0(void)
{

		while(1) {
			LED_BLUE_ON();
			lidar_data[0] = lidar_GetMeasurement(0);
			LED_BLUE_OFF();
			OS_bWait(&serial);
			printf("lidar 0: %d mm\n\r", lidar_data[0]);
			OS_bSignal(&serial);
		}
}

void lidar_task1(void)
{
			while(1) {
			LED_GREEN_ON();
			lidar_data[1] = lidar_GetMeasurement(1);
			LED_GREEN_OFF();
				OS_bWait(&serial);
			printf("lidar 1: %d mm\n\r", lidar_data[1]);
			OS_bSignal(&serial);
			}
}

int lidar_Init(void)
{
 
		I2C_Init();               // must initialize I2C before initialize VL53L0X

  	if (VL53L0X_Init(VL53L0X_I2C_ADDR, 0))
    {
        OS_AddThread(&lidar_task0, 128, 1);
    }
    else
    { 		
				LED_BLUE_TOGGLE();
        return -1;
    }
		
	if (VL53L0X_Init(VL53L0X_I2C_ADDR, 1))
	{
			OS_AddThread(&lidar_task1, 128, 1);
	}
	else
	{   
		  LED_GREEN_TOGGLE();
			return -1;
	}
	
		OS_InitSemaphore(&serial, 1);
			return 0;
}

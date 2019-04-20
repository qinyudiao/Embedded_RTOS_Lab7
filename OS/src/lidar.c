
#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "I2C.h"
#include "VL53L0X.h"
#include "UART.h"
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LED.h"

#define ADC_CLK_FREQ (80000000)
static int dis; 

// void Timer1_Init(uint32_t fs, int priority) {
//     uint32_t period = ADC_CLK_FREQ / fs;

//     SYSCTL_RCGCTIMER_R |= 0x02;                                  // 0) activate TIMER1
//     TIMER1_CTL_R = 0x00000000;                                   // 1) disable TIMER1A during setup
//     TIMER1_CFG_R = 0x00000000;                                   // 2) configure for 32-bit mode
//     TIMER1_TAMR_R = 0x00000002;                                  // 3) configure for periodic mode, default down-count settings
//     TIMER1_TAILR_R = period - 1;                                 // 4) reload value
//     TIMER1_TAPR_R = 0;                                           // 5) bus clock resolution
//     TIMER1_ICR_R = 0x00000001;                                   // 6) clear TIMER1A timeout flag
//     TIMER1_IMR_R = 0x00000001;                                   // 7) arm timeout interrupt
//     NVIC_PRI5_R = (NVIC_PRI5_R & 0xFFFF00FF) | (priority << 13); // 8) priority bit 15-13
//     // interrupts enabled in the main program after all devices initialized
//     // vector number 37, interrupt number 21
//     NVIC_EN0_R = 1 << 21;      // 9) enable IRQ 21 in NVIC
//     TIMER1_CTL_R = 0x00000001; // 10) enable TIMER1A
// }

int LiDAR_GetMeasurement(void)
{
    VL53L0X_RangingMeasurementData_t measurement;
    VL53L0X_getSingleRangingMeasurement(&measurement, 0);
    if (measurement.RangeStatus != 4)
    {
        return measurement.RangeMilliMeter;
    }
    else    // out of range
        return -1;
}

static int len = 0;

void LiDAR_test(void) {
   // if (len++ < 20)
       printf("distance: %d mm\n\r", dis);
    OS_Kill();
}

void LiDAR_task(void) {
    dis = LiDAR_GetMeasurement();
    if (dis != -1) {
        OS_AddThread(&LiDAR_test, 128, 1);
		}
}

int LiDAR_Init(void)
{
    // init and wake up VL53L0X
    if (VL53L0X_Init(VL53L0X_I2C_ADDR, 0)) {
        // Timer1_Init(10, 2);
        unsigned long fs = 10;
        unsigned long period = ADC_CLK_FREQ / fs;
        OS_AddPeriodicThread(&LiDAR_task, period, 2);
        return 0;
    }
    else {
        return -1;
		}
}

// void Timer1A_Handler(void){
// 	TIMER1_ICR_R = TIMER_ICR_TATOCINT;  // acknowledge

// }
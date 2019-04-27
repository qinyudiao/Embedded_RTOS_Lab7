//bumperSwitch.c
//J2;PC6;right
//J3;PC7;left

#include "tm4c123gh6pm.h"
#include <stdio.h>
#include <stdint.h>
#include "OS.h"

static long lastPC6, lastPC7;

long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value

extern uint8_t left_bumper_state; // 0:none detected 1:detected
extern uint8_t right_bumper_state;

static void (*left_bumper_task)(void);
static void (*right_bumper_task)(void);
static int left_bumper_pri;
static int right_bumper_pri;
#define PC6 (*((volatile unsigned long *)0x40006100))
#define PC7 (*((volatile unsigned long *)0x40006200))


static void right_bumper_debounce(void) {
	OS_Sleep(10);
	lastPC6 = PC6 & 0x40;		// lastPC7 reflects the state of switch after debounce
	GPIO_PORTC_ICR_R = 0x40;
	GPIO_PORTC_IM_R |= 0x40;
	OS_Kill();	// only one time usage, so kill right away
}

static void left_bumper_debounce(void) {
	OS_Sleep(10);
	lastPC7 = PC7 & 0x80;
	GPIO_PORTC_ICR_R = 0x80;
	GPIO_PORTC_IM_R |= 0x80;
	OS_Kill();
}

int OS_AddRightBumperTask(void(*task)(void), unsigned long priority) {
	long sr = StartCritical();
	if (!right_bumper_task) {
		right_bumper_task = task;
		right_bumper_pri = priority;
		unsigned long volatile delay;
		SYSCTL_RCGCGPIO_R |= 0x00000004; 	// (a) activate clock for port C
		delay = SYSCTL_RCGCGPIO_R;
		GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;		// unlock port C
		GPIO_PORTC_CR_R |= 0x40;           // allow changes to PC6
		GPIO_PORTC_DIR_R &= ~0x40;    		// (c) make PC6 in (built-in button)
		GPIO_PORTC_AFSEL_R &= ~0x40;  		//     disable alt funct on PC6
		GPIO_PORTC_DEN_R |= 0x40;     		//     enable digital I/O on PC6
		GPIO_PORTC_PCTL_R &= ~0x0F000000; // configure PC6 as GPIO
		GPIO_PORTC_AMSEL_R = 0;       		//     disable analog functionality on PC
		GPIO_PORTC_PUR_R |= 0x40;     		//     enable weak pull-up on PC6
		GPIO_PORTC_IS_R &= ~0x40;     		// (d) PC6 is edge-sensitive
		GPIO_PORTC_IBE_R |= 0x40;     		//     PC6 is on both edges
											// debounce requires 2 edges: press and release
		GPIO_PORTC_ICR_R = 0x40;      		// (e) clear flag4
		GPIO_PORTC_IM_R |= 0x40;      		// (f) arm interrupt on PC6
		NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF) | ((priority & 0x07) << 21);  // set up priority
		NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC
		lastPC6 = PC6 & 0x40;				// initially high
		EndCritical(sr);
		return 1;
	} else {
		EndCritical(sr);
		return 0;
	}
}

int OS_AddLeftBumperTask(void(*task)(void), unsigned long priority) {
	long sr = StartCritical();
	if (!left_bumper_task) {
		left_bumper_task = task;
		left_bumper_pri = priority;
		unsigned long volatile delay;
		SYSCTL_RCGCGPIO_R |= 0x00000004; 	// (a) activate clock for port C
		delay = SYSCTL_RCGCGPIO_R;
		GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;		// unlock port C
		GPIO_PORTC_CR_R |= 0x80;           // allow changes to PC7
		GPIO_PORTC_DIR_R &= ~0x80;    		// (c) make PC7 in (built-in button)
		GPIO_PORTC_AFSEL_R &= ~0x80;  		//     disable alt funct on PC7
		GPIO_PORTC_DEN_R |= 0x80;     		//     enable digital I/O on PC7
		GPIO_PORTC_PCTL_R &= ~0xF0000000; // configure PC7 as GPIO
		GPIO_PORTC_AMSEL_R = 0;       		//     disable analog functionality on PC
		GPIO_PORTC_PUR_R |= 0x80;     		//     enable weak pull-up on PC7
		GPIO_PORTC_IS_R &= ~0x80;     		// (d) PC7 is edge-sensitive
		GPIO_PORTC_IBE_R |= 0x80;     		//     PC7 is on both edges
											// debounce requires 2 edges: press and release
		GPIO_PORTC_ICR_R = 0x80;      		// (e) clear flag4
		GPIO_PORTC_IM_R |= 0x80;      		// (f) arm interrupt on PC7
		NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF) | ((priority & 0x07) << 21);  // set up priority
		NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC
		lastPC7 = PC7 & 0x80;				// initially high
		EndCritical(sr);
		return 1;
	} else {
		EndCritical(sr);
		return 0;
	}
}

void GPIOPortC_Handler(void){
	 long sr = StartCritical();;
	// ********************************When PC6 is pressed********************************
   if (GPIO_PORTC_RIS_R&0x40) { //PC6
        GPIO_PORTC_IM_R &= ~0x40;     // disarm interrupt on PC6
		 		if (lastPC6) {
					right_bumper_task();
				}
		 		// debounce required on both press and release
				int ret = OS_AddThread(right_bumper_debounce, 128, 1);  // for debounce purpose, priority for switch tasks need to be high
				if (ret == 0) {  // failed, arm right away			 // so that it can be scheduled right away
					GPIO_PORTC_ICR_R = 0x40;
					GPIO_PORTC_IM_R |= 0x40;
				}
	 }
	// ********************************When PC7 is pressed********************************
	 if (GPIO_PORTC_RIS_R&0x80){
				GPIO_PORTC_IM_R &= ~0x08;     // disarm interrupt on PC7
		 		if (lastPC7) {
					left_bumper_task();
				}
		 		// debounce required on both press and release
				int ret = OS_AddThread(left_bumper_debounce, 128, 1);  // for debounce purpose, priority for switch tasks need to be high
				if (ret == 0) {  // failed, arm right away			 // so that it can be scheduled right away
					GPIO_PORTC_ICR_R = 0x80;
					GPIO_PORTC_IM_R |= 0x80;
				}
	 }
	 EndCritical(sr);
 }
	 
	 
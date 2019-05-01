// ***** 0. Documentation Section *****
// TableTrafficLight.c for (Lab 10 edX), Lab 5 EE319K
// Runs on LM4F120/TM4C123
// Program written by: Qinyu Diao, Yue Shen
// Date Created: 1/24/2015 
// Last Modified: 3/7/2016 
// Section 3-4pm     TA: 
// Lab number: 5
// Hardware connections
// *********************************************************
// west red light connected to PE2
// west yellow light connected to PE1
// west green light connected to PE0
// south facing red light connected to PE5
// south facing yellow light connected to PE4
// south facing green light connected to PE3
// pedestrian detector connected to PA4 (1=pedestrian present)
// south car detector connected to PA3 (1=car present)
// west car detector connected to PA2 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include <stdint.h>
#include "SysTick.h"
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// name the states
#define GoW	0
#define	SdW	1
#define	GoS	2
#define	SdS	3
#define	sPw	4
#define	sPs	5
#define GoPw	6
#define GoPs	7
#define wFlash1 8
#define	wFlash2 9
#define wFlash3 10
#define wFlash4 11
#define wFlash5 12
#define sFlash1 13
#define sFlash2	14
#define sFlash3	15
#define sFlash4	16
#define sFlash5	17


// ***** 2. Global Declarations Section *****
struct State{
		//output
		uint8_t PE;
		uint8_t PF;
		//time 100 = 1s
		uint32_t time;
		//array of next state; elements are state numbers, index are inputs(3 bits)
		uint8_t next[8];

};

struct State FSM[18]={
		{0x21, 0x02, 200, {GoW, GoW, SdW, SdW, sPw, sPw, sPw, sPw}},																	//GoW 0
		{0x22, 0x02, 80,  {GoS, GoS, GoS, GoS, GoS, GoS, GoS, GoS}},																	//SdW	1
		{0x0C, 0x02, 200, {GoS, SdS, GoS, SdS, sPs, sPs, sPs, sPs}},																	//GoS 2
		{0x14, 0x02, 80,  {GoW, GoW, GoW, GoW, GoW, GoW, GoW, GoW}},																	//SdS	3
		{0x22, 0x02, 80,  {GoPw, GoPw, GoPw, GoPw, GoPw, GoPw, GoPw, GoPw}},													//sPw 4
		{0x14, 0x02, 80,  {GoPs, GoPs, GoPs, GoPs, GoPs, GoPs, GoPs, GoPs}},													//sPs	5
		{0x24, 0x08, 200, {wFlash1, wFlash1, wFlash1, wFlash1, wFlash1, wFlash1, wFlash1, wFlash1}},	//GoPw	6
		{0x24, 0x08, 200, {sFlash1, sFlash1, sFlash1, sFlash1, sFlash1, sFlash1, sFlash1, sFlash1}},	//GoPs	7
		{0x24, 0x02, 20,  {wFlash2, wFlash2, wFlash2, wFlash2, wFlash2, wFlash2, wFlash2, wFlash2}},	//wFlash1	8
		{0x24, 0x00, 20,  {wFlash3, wFlash3, wFlash3, wFlash3, wFlash3, wFlash3, wFlash3, wFlash3}},	//wFlash2	9
		{0x24, 0x02, 20,  {wFlash4, wFlash4, wFlash4, wFlash4, wFlash4, wFlash4, wFlash4, wFlash4}},	//wFlash3	10
		{0x24, 0x00, 20,  {wFlash5, wFlash5, wFlash5, wFlash5, wFlash5, wFlash5, wFlash5, wFlash5}},	//wFlash4	11
		{0x24, 0x02, 20,  {GoS, GoW, GoS, GoS, GoS, GoW, GoS, GoS}},																	//wFlash5	12
		{0x24, 0x02, 20,  {sFlash2, sFlash2, sFlash2, sFlash2, sFlash2, sFlash2, sFlash2, sFlash2}},	//sFlash1	13
		{0x24, 0x00, 20,  {sFlash3, sFlash3, sFlash3, sFlash3, sFlash3, sFlash3, sFlash3, sFlash3}},	//sFlash2	14
		{0x24, 0x02, 20,  {sFlash4, sFlash4, sFlash4, sFlash4, sFlash4, sFlash4, sFlash4, sFlash4}},	//sFlash3	15
		{0x24, 0x00, 20,  {sFlash5, sFlash5, sFlash5, sFlash5, sFlash5, sFlash5, sFlash5, sFlash5}},	//sFlash4	16
		{0x24, 0x02, 20,  {GoW, GoW, GoS, GoW, GoW, GoW, GoS, GoW}},																	//sFlash5	17
};
// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

int realmain(void){ 
	//initializations
	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
	SysTick_Init();
	// Port A initialization
	volatile uint32_t delay;
	SYSCTL_RCGC2_R |= 0x00000001;									//port A
	delay = SYSCTL_RCGC2_R;											//NOP
	GPIO_PORTA_DEN_R|=0x1C;												//We are using PA 2-4 as inputs
	GPIO_PORTA_DIR_R&=0xE3;												//PA2 = W sensor, PA3 = S sensor, PA4 = pedestrian button
	// Port E initialization
	SYSCTL_RCGC2_R |= 0x00000010;									//port E
	delay = SYSCTL_RCGC2_R;											//NOP
	GPIO_PORTE_DEN_R|=0x3F;												//We are using PE 0-5 as traffic lights outputs
	GPIO_PORTE_DIR_R|=0x3F;												//PE 0-5 according Rw, Yw, Gw, Rs, Ys, Gs, Pg, Pr
	// Port F initialization
	SYSCTL_RCGC2_R |= 0x00000020;									//port F clock
	delay = SYSCTL_RCGC2_R;											//NOP
	GPIO_PORTF_DEN_R|=0x0A;												//PF3 for Walk light, PF1 for Don't Walk light
	GPIO_PORTF_DIR_R|=0x0A;
	
	uint8_t state = GoW;
  
  EnableInterrupts();
  while(1){
				//output
		GPIO_PORTE_DATA_R = FSM[state].PE;
		GPIO_PORTF_DATA_R = FSM[state].PF;
				//delay
		SysTick_Wait10ms(FSM[state].time);
		uint8_t input = (GPIO_PORTA_DATA_R>>2)&0x07;
				//next state
		state =FSM[state].next[input];		
  }
}



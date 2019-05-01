

//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
//#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
//#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
//#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
//#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
//#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
//#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
unsigned long in,out;

// ***** 1. Pre-processor Directives Section *****
#include <stdint.h>
#include <stdio.h>
#include "SysTick.h"
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

// PE0, PB0, or PA2 connected to positive logic momentary switch using 10 k ohm pull down resistor
// PE1, PB1, or PA3 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).
void delay100ms(unsigned long numOf100msDelays)
{
	unsigned long i;
	while (numOf100msDelays > 0)
	{
		i = 1333333; //this number means 100ms
		while (i > 0)
		{
			i = i - 1;
		}
		//decrements every 100ms
		numOf100msDelays = numOf100msDelays - 1; 
	}
}



int main(void){
  unsigned long volatile delay;
	unsigned long dataRegIn = 0;
	
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  //TExaS_Init(SW_PIN_PE0, LED_PIN_PE1);  // activate grader and set system clock to 80 MHz
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210);
	
  EnableInterrupts();           // enable interrupts for the grader
  SYSCTL_RCGC2_R |= 0x10;           // Port E clock
  delay = SYSCTL_RCGC2_R;           // wait 3-5 bus cycles
  GPIO_PORTE_DIR_R |= 0x02;         // PE1 output
  GPIO_PORTE_DIR_R &= ~0x01;        // PE0 input 
  GPIO_PORTE_AFSEL_R &= ~0x03;      // not alternative
  GPIO_PORTE_AMSEL_R &= ~0x03;      // no analog
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // bits for PE1, PE0
  GPIO_PORTE_DEN_R |= 0x03;         // enable PE1, PE0
	
	GPIO_PORTE_DATA_R |= 0x02; //start with LED on (PE1 = 1)
	
	SYSCTL_RCGC2_R |= 0x04;           // Port C clock
  delay = SYSCTL_RCGC2_R;           // wait 3-5 bus cycles
  GPIO_PORTC_DIR_R |= 0x40;         // PC6 output
  GPIO_PORTC_DIR_R &= ~0x80;        // PC7 input 
  GPIO_PORTC_AFSEL_R &= ~0xC0;      // not alternative
  GPIO_PORTC_AMSEL_R &= ~0xC0;      // no analog
  GPIO_PORTC_PCTL_R &= ~0xFF000000; // bits for PE1, PE0
  GPIO_PORTC_DEN_R |= 0xC0;         // enable PE1, PE0
	
	GPIO_PORTC_DATA_R |= 0x40; //start with LED on (PE1 = 1)
	while(1){
    delay100ms(1); //delay for (1) 100 ms interval
    in = (GPIO_PORTC_DATA_R&0x80); // in 0 if not pressed, 1 if pressed
		//If switch pressed (PE0=1), toggle LED once, else turn LED ON
    //out = (in xor 0x01) << 1 (shift to PE1 LED output)
		//so out = 1 if switch NOT pressed (0 xor 1 = 1 = LED ON)
		//and out = 0 if switch is pressed (1 xor 1 = 0)
		//this works since you can assume LED on all the time if
		//switch not pressed, so to toggle it, just xor it with in = 1
		//to toggle it off when switch pressed
		//did NOT work!  LED did not toggle
    //out = (in^0x01)<<1;   // out 2 if not pressed, 0 if switch pressed
    if (in == 0x80) //PE0 = switch = pressed
		{
			dataRegIn = GPIO_PORTC_DATA_R & 0x40;
			if (dataRegIn == 0) //PF2 = 0 = LED off
				{ //LED off, toggle LED once (turn it on)
					GPIO_PORTC_DATA_R |= 0x40; //turn LED on
				}
				else //LED on already
				{ //LED on, toggle LED once (turn off)
					GPIO_PORTC_DATA_R &= ~(0x40); //turn LED off
				}
		}
	else
			GPIO_PORTC_DATA_R |= 0x40; //turn LED on
	}
}

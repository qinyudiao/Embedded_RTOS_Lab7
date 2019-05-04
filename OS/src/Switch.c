// Switch.c
// Runs on TMC4C123
// Use GPIO in edge time mode to request interrupts on any
// edge of PF4 and start Timer0B. In Timer0B one-shot
// interrupts, record the state of the switch once it has stopped
// bouncing.
// Daniel Valvano
// May 3, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// PF4 connected to a negative logic switch using internal pull-up (trigger on both edges)
#include <stdint.h>
#include "tm4c123gh6pm.h"

#define PF0 (*((volatile unsigned long *)0x40025004))
#define PF1 (*((volatile unsigned long *)0x40025008))
#define PF2 (*((volatile unsigned long *)0x40025010))
#define PF3 (*((volatile unsigned long *)0x40025020))
#define PF4 (*((volatile uint32_t *)0x40025040))
#define PC6 (*((volatile uint32_t *)0x40006100))
#define PC7 (*((volatile uint32_t *)0x40006200))

volatile static unsigned long Touch;   // true on touch
volatile static unsigned long Release; // true on release
volatile static unsigned long Last;    // previous
void (*TouchTask)(void);               // user function to be executed on touch
void (*ReleaseTask)(void);             // user function to be executed on release

//FOR SW2
volatile static unsigned long Touch2;   // true on touch
volatile static unsigned long Release2; // true on release
volatile static unsigned long Last2;    // previous
void (*TouchTask2)(void);               // user function to be executed on touch
void (*ReleaseTask2)(void);             // user function to be executed on release

//FOR PC7
volatile static unsigned long Touch7;   // true on touch
volatile static unsigned long Release7; // true on release
volatile static unsigned long LastPC7;    // previous
void (*TouchTask7)(void);               // user function to be executed on touch
void (*ReleaseTask7)(void);             // user function to be executed on release

//FOR PC6
volatile static unsigned long Touch6;   // true on touch
volatile static unsigned long Release6; // true on release
volatile static unsigned long LastPC6;    // previous
void (*TouchTask6)(void);               // user function to be executed on touch
void (*ReleaseTask6)(void);             // user function to be executed on release

static void Timer0Arm(void)
{
  TIMER0_CTL_R = 0x00000000;                             // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;                             // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x0000001;                             // 3) 1-SHOT mode
  TIMER0_TAILR_R = 160000;                               // 4) 10ms reload value
  TIMER0_TAPR_R = 0;                                     // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;                             // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;                             // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R & 0x00FFFFFF) | 0x80000000; // 8) priority 4
                                                         // interrupts enabled in the main program after all devices initialized
                                                         // vector number 35, interrupt number 19
  NVIC_EN0_R |= 1 << 19;                                  // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;                             // 10) enable TIMER0A
}
static void GPIOArm(void)
{
  GPIO_PORTF_ICR_R = 0x11;                               // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;                               // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;                               // (h) enable interrupt 30 in NVIC
}

static void GPIO_PORTC_Arm(void)
{
  GPIO_PORTC_ICR_R = 0xC0;                               // (e) clear flag 6-7
  GPIO_PORTC_IM_R |= 0xC0;                               // (f) arm interrupt on PC6-7 *** No IME bit as mentioned in Book ***
  NVIC_PRI0_R = (NVIC_PRI0_R & 0xFF00FFFF) | 0x00800000; // (g) priority 4
  NVIC_EN0_R |= 0x00000004;                               // (h) enable interrupt 2 in NVIC
}
// Initialize switch interface on PF4 and PF0. Have to use PF4 first, and PF0 otherwise can't work.
// Inputs:  pointer to a function to call on touch (falling edge),
//          pointer to a function to call on release (rising edge)
//
// Outputs: none
void Switch_Init(void (*touchtask)(void), void (*releasetask)(void))
{
  // **** general initialization ****
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  while ((SYSCTL_PRGPIO_R & 0x00000020) == 0)
  {
  };
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTF_CR_R |= 0x11;
  GPIO_PORTF_DIR_R &= ~0x11;   // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11; //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x11;    //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R & 0xFFF0FFF0) + 0x00000000;
  // 6) disable alt funct on PF0 and PF4
  GPIO_PORTF_AMSEL_R &= ~0x11; //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;    //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;    // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x11;    //     PF4 is both edges

  /*
	GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF0 in (built-in button)
	GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF0
  GPIO_PORTF_PCTL_R &= ~0x0000000F; // configure PF0 as GPIO
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x01;     //     PF0 is both edges
		*/

  GPIOArm();

  SYSCTL_RCGCTIMER_R |= 0x01; // 0) activate TIMER0
  TouchTask = touchtask;      // user function
  ReleaseTask = releasetask;  // user function
  Touch = 0;                  // allow time to finish activating
  Release = 0;
  Last = PF4; // initial switch state
}

void Right_Bumper_Init(void (*touchtask)(void), void (*releasetask)(void))
{
  // **** general initialization ****
	SYSCTL_RCGCGPIO_R |= 0x04;           // Port C clock
  unsigned long delay = SYSCTL_RCGCGPIO_R ;           // wait 3-5 bus cycles
	GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTC_DIR_R &= ~0xC0;        // PC6-7 input 
  GPIO_PORTC_AFSEL_R &= ~0xC0;      // not alternative
  GPIO_PORTC_AMSEL_R &= ~0xC0;      // no analog
  GPIO_PORTC_PCTL_R &= ~0xFF000000; // bits for PC7, PC6
  GPIO_PORTC_DEN_R |= 0xC0;         // enable PC6, PC7
  GPIO_PORTC_CR_R |= 0xC0;
  //GPIO_PORTC_PUR_R |= 0xC0;    //     enable weak pull-up on PC6-7
  GPIO_PORTC_IS_R &= ~0xC0;    // (d) PC6-7 is edge-sensitive
  GPIO_PORTC_IBE_R |= 0xC0;    //     PC6-7 is both edges

  GPIO_PORTC_Arm();

  SYSCTL_RCGCTIMER_R |= 0x01; // 0) activate TIMER0
  TouchTask6 = touchtask;      // user function
  ReleaseTask6 = releasetask;  // user function
  Touch6 = 0;                  // allow time to finish activating
  Release6 = 0;
  LastPC6 = PC6; // initial switch state
}

void Switch2_Init(void (*touchtask)(void), void (*releasetask)(void))
{

  TouchTask2 = touchtask;     // user function
  ReleaseTask2 = releasetask; // user function
  Touch2 = 0;                 // allow time to finish activating
  Release2 = 0;
  Last2 = PF0; // initial switch state
}

void Left_Bumper_Init(void (*touchtask)(void), void (*releasetask)(void))
{

  TouchTask7 = touchtask;     // user function
  ReleaseTask7 = releasetask; // user function
  Touch7 = 0;                 // allow time to finish activating
  Release7 = 0;
  LastPC7 = PC7; // initial switch state
}

// Interrupt on rising or falling edge of PF4 (CCP0)
void GPIOPortF_Handler(void)
{
  GPIO_PORTF_IM_R &= ~0x11; // disarm interrupt on PF4,PF0

  if (Last == 0x10)
  { // 0x10 means it was previously released

    if (GPIO_PORTF_DATA_R == 0x01)
    {
      Touch = 1; // touch occurred
      if (TouchTask)
      {
        TouchTask(); // execute user task
      }
    }
  }
  else
  {
    Release = 1; // release occurred
    if (ReleaseTask != 0)
    {
      ReleaseTask(); // execute user task
    }
  }

  if (Last2 == 0x01)
  { // 0x10 means it was previously released
    if (GPIO_PORTF_DATA_R == 0x10)
    {
      Touch2 = 1; // touch occurred
      if (TouchTask2)
      {
        TouchTask2(); // execute user task
      }
    }
  }
  else
  {
    Release2 = 1; // release occurred
    if (ReleaseTask2 != 0)
    {
      ReleaseTask2(); // execute user task
    }
  }

  Timer0Arm(); // start one shot
}

void GPIOPortC_Handler(void)
{
  GPIO_PORTC_IM_R &= ~0xC0; // disarm interrupt on PC6-7
	
  if ((LastPC7&0x80) == 0x80)
  { // 0x10 means it was previously released

    if ((GPIO_PORTC_DATA_R&0x80) ==0x80){
      Touch7 = 1; // touch occurred
      if (TouchTask7)
      {
        TouchTask7(); // execute user task
      }
    }
  }
  else
  {
    Release7 = 1; // release occurred
    if (ReleaseTask7 != 0)
    {
      ReleaseTask7(); // execute user task
    }
  }

  if ((LastPC6&0x40) == 0x40)
  { // 0x10 means it was previously released
    if ((GPIO_PORTC_DATA_R&0x40) == 0x40)
    {
      Touch6 = 1; // touch occurred
      if (TouchTask6)
      {
        TouchTask6(); // execute user task
      }
    }
  }
  else
  {
    Release6 = 1; // release occurred
    if (ReleaseTask6 != 0)
    {
      ReleaseTask6(); // execute user task
    }
  }

  Timer0Arm(); // start one shot
}
// Interrupt 10 ms after rising edge of PF4
void Timer0A_Handler(void)
{
  TIMER0_IMR_R = 0x00000000; // disarm timeout interrupt
  Last = PF4;                // switch state
  Last2 = PF0;
	LastPC6 = PC6;                // switch state
  LastPC7 = PC7;
  GPIO_PORTC_Arm();
	GPIOArm(); // start GPIO
}

// Wait for switch to be pressed
// There will be minimum time delay from touch to when this function returns
// Inputs:  none
// Outputs: none
void Switch_WaitPress(void)
{
  while (Touch == 0)
  {
  };         // wait for press
  Touch = 0; // set up for next time
}

// Wait for switch to be released
// There will be minimum time delay from release to when this function returns
// Inputs:  none
// Outputs: none
void Switch_WaitRelease(void)
{
  while (Release == 0)
  {
  };           // wait
  Release = 0; // set up for next time
}

// Return current value of the switch
// Repeated calls to this function may bounce during touch and release events
// If you need to wait for the switch, use WaitPress or WaitRelease
// Inputs:  none
// Outputs: false if switch currently pressed, true if released
unsigned long Switch_Input(void)
{
  return PF4;
}

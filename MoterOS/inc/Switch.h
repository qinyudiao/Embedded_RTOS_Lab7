// Switch.h
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

/**
 * @brief Initialize switch interface on PF4 (SW1)
 * @param touchtask    pointer to a function to call on touch (falling edge),
 * @param releasetask  pointer to a function to call on release (rising edge)
*/
void Switch_Init(void(*touchtask)(void), void(*releasetask)(void));

/**
 * @brief Initialize switch interface on PF0 (SW2)
 * @param touchtask    pointer to a function to call on touch (falling edge),
 * @param releasetask  pointer to a function to call on release (rising edge)
*/
void Switch2_Init(void(*touchtask)(void), void(*releasetask)(void));

/**
 *  Wait for switch to be pressed 
 *  There will be minimum time delay from touch to when this function returns
 *  Inputs:  none
*/
void Switch_WaitPress(void);

/**
 *  Wait for switch to be released 
 *  There will be minimum time delay from release to when this function returns
 *  Inputs:  none
*/
void Switch_WaitRelease(void);

/**
 *  @brief Return current value of the switch 
 *         Repeated calls to this function may bounce during touch and release events
 *         If you need to wait for the switch, use WaitPress or WaitRelease
 *  @return false if switch currently pressed, true if released 
*/
unsigned long Switch_Input(void);

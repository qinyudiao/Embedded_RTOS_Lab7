//*****************************************************************************
//
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.6
// You may use, edit, run or distribute this file
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task
// PF4 is SW1 button input

// Analog inputs
// PE3 sequencer 3, channel 3, J8/PE0, sampling in DAS(), software start
// PE0 timer-triggered sampling, channel 0, J5/PE3, 50 Hz, processed by Producer
//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include "OS.h"
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include "interpreter.h"
#include "servo.h"
#include "ff.h"
#include "diskio.h"
#include "motors.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "can0.h"

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);

#define PF0 (*((volatile unsigned long *)0x40025004))
#define PF1 (*((volatile unsigned long *)0x40025008))
#define PF2 (*((volatile unsigned long *)0x40025010))
#define PF3 (*((volatile unsigned long *)0x40025020))
#define PF4 (*((volatile unsigned long *)0x40025040))

#define PD0 (*((volatile unsigned long *)0x40007004))
#define PD1 (*((volatile unsigned long *)0x40007008))
#define PD2 (*((volatile unsigned long *)0x40007010))
#define PD3 (*((volatile unsigned long *)0x40007020))

#define TIMESLICE 2 * TIME_1MS // thread switch time in system time units

int NumCreated = 0;


//*******************lab 4 main **********
int realmain(void)
{               // lab 4 real main
  OS_Init();    // initialize, disable interrupts
  
  //********initialize communication channels
  OS_Fifo_Init(256);
  
  ADC_Init(3);                                  // sequencer 3, channel 3, PE0, sampling in DAS()
  
  //NumCreated += OS_AddThread(&IdleTask, 128, 7); // runs when nothing useful to do

  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//*****************test project 2*************************

//******************* test main2 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread

#define PROG_STEPS (20)

void motor_task(void)
{
  static int step = 1;
  static int torque = -50;
  // Motors_SetTorque(torque, torque);
  Motors_SetTorque(torque, torque);
  torque += step;
  if((torque > 50) || (torque < -50))
  {
    step = -step;
    torque += step;
  }
}

void servo_task(void)
{
  static int angle = 0;
  Servo_SetAngle(angle);
  angle += 18;
  if(angle > 180)
  {
    angle = 0;
  }

}

void Right_motor_task(void)
{
  int data = 0;
  while(1)
  {
    CAN0_GetMailwithIdx((uint8_t *)&data,0);
    Motors_SetTorque_Right(data);
  }
}

void Left_motor_task(void)
{
  int data = 0;
  while(1)
  {
    CAN0_GetMailwithIdx((uint8_t *)&data,1);
    Motors_SetTorque_Left(data);
  }
}

void Servo_motor_task(void)
{
  int data = 0;
  while(1)
  {
    CAN0_GetMailwithIdx((uint8_t *)&data,2);
    Servo_SetAngle(data);
  }  
}

void Left_Right_motor_task(void)
{
  int data = 0;
  int Left = 0;
  int Right = 0;
  while(1)
  {
    CAN0_GetMailwithIdx((uint8_t *)&data,3);
    Left = (data & (0xFFFF0000)) >> 16;
    Right = (data &(0xFFFF));
    Motors_SetTorque(Left,Right);
  }  
}

int motor_main(void)
{
  OS_Init(); // initialize, disable interrupts
  Motors_Init();
  Servo_Init();
  CAN0_Open();
  CAN0_SetRecv(0); // 0 : Right control
  CAN0_SetRecv(1); // 1 : Left control
  CAN0_SetRecv(2); // 2 : Servo control
  CAN0_SetRecv(3); // 3 : Left and Right control, First two byte for Left, second for Right
  NumCreated = 0;
  
  // create initial foreground threads
//  NumCreated += OS_AddPeriodicThread(&motor_task, 10 * TIME_1MS, 2);
// NumCreated += OS_AddPeriodicThread(&servo_task, 1000 * TIME_1MS, 2);
  NumCreated += OS_AddThread(&Right_motor_task,128,1);
  NumCreated += OS_AddThread(&Left_motor_task,128,1);
  NumCreated += OS_AddThread(&Servo_motor_task,128,1);
  NumCreated += OS_AddThread(&Left_Right_motor_task,128,1);
  

  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

void can0_task(void)
{
  uint8_t data[4];
  static int i = 1024;
  data[0] = i%10;
  data[1] = i/10;
  data[2] = i/100;
  data[3] = i/1000;
  CAN0_SendDatawithIdx(data,6);
}
void can0_recvtask(void)
{
  uint8_t data[4];
  char str[64];
  while(1){
    CAN0_GetMailwithIdx(data,6);
    sprintf(str, "recv data %d %d %d %d\r\n", data[0],data[1],data[2],data[3]);
    UART_OutString(str);
  }
}

int can0_testmain()
{
  OS_Init();
  CAN0_Open();
  NumCreated = 0;
  CAN0_SetRecv(6);
  // create initial foreground threads
 // NumCreated += OS_AddPeriodicThread(&motor_task, 1 * TIME_1MS, 2);
  NumCreated += OS_AddPeriodicThread(&can0_task, 1 * TIME_1MS,1);
  //NumCreated += OS_AddThread(&can0_recvtask, 128, 2);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}

// Main stub
int main(void)
{
  return motor_main();
}

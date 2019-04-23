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
#include "motorcan.h"
#include "can0.h"
#include "IR.h"


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

#define ANGLELEFT_OFFSET 200
#define ANGELRIGHT_OFFSET 200
#define LEFT_OFFSET  105
#define RIGHT_OFFSET 105
#define cosTHETA (5255)
// cosTHETA * 1000 value
#define KP 1
#define KD 1
#define KI 1

static FATFS g_sFatFs;

void PortD_Init(void)
{
  SYSCTL_RCGCGPIO_R |= 0x08; // activate port D
  while ((SYSCTL_PRGPIO_R & 0x08) == 0)
  {
  };
  GPIO_PORTD_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F; // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;    // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F; // disable analog functionality on PD
}
unsigned long NumCreated; // number of foreground threads created
unsigned long NumSamples; // incremented every sample
unsigned long DataLost;   // data sent by Producer, but not received by Consumer
unsigned long PIDWork;    // current number of PID calculations finished
unsigned long FilterWork; // number of digital filter calculations finished

int Running; // true while robot is running

#define TIMESLICE 2 * TIME_1MS // thread switch time in system time units
long x[64], y[64];             // input and output arrays for FFT
Sema4Type doFFT;               // set every 64 samples by DAS

long median(long u1, long u2, long u3)
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
//------------ADC2millimeter------------
// convert 12-bit ADC to distance in 1mm
// it is known the expected range is 100 to 800 mm
// Input:  adcSample 0 to 4095
// Output: distance in 1mm
long ADC2millimeter(long adcSample)
{
  if (adcSample < 494)
    return 799; // maximum distance 80cm
  return (268130 / (adcSample - 159));
}
long Distance3;  // distance in mm on IR3
uint32_t Index3; // counts to 64 samples
long x1, x2, x3;
void DAS(void)
{
  long output;
  //PD0 ^= 0x01;
  x3 = x2;
  x2 = x1;       // MACQ
  x1 = ADC_In(); // channel set when calling ADC_Init
 //PD0 ^= 0x01;
  if (Index3 < 64)
  {
    output = median(x1, x2, x3); // 3-wide median filter
    Distance3 = ADC2millimeter(output);
    FilterWork++; // calculation finished
    x[Index3] = Distance3;
    Index3++;
    if (Index3 == 64)
    {
      OS_Signal(&doFFT);
    }
  }
 // PD0 ^= 0x01;
}
void DSP(void)
{
  unsigned long DCcomponent; // 12-bit raw ADC sample, 0 to 4095
  //OS_InitSemaphore(&doFFT, 0);
  while (1)
  {
    OS_Wait(&doFFT); // wait for 64 samples
    //PD2 = 0x04;
    cr4_fft_64_stm32(y, x, 64); // complex FFT of last 64 ADC values
    //PD2 = 0x00;
    Index3 = 0;                  // take another buffer
    DCcomponent = y[0] & 0xFFFF; // Real part at frequency 0, imaginary part should be zero

    ST7735_Message(1, 0, "IR3 (mm) =", DCcomponent);
  }
}
char Name[8] = "robot0";
//******** Robot ***************
// foreground thread, accepts data from producer
// inputs:  none
// outputs: none
void Robot(void)
{
  unsigned long data;     // ADC sample, 0 to 1023
  unsigned long voltage;  // in mV,      0 to 3300
  unsigned long distance; // in mm,      100 to 800
  unsigned long time;     // in 10msec,  0 to 1000
  OS_ClearMsTime();
  DataLost = 0; // new run with no lost data
  //OS_Fifo_Init(256);
  printf("Robot running...");
  //eFile_RedirectToFile(Name); // robot0, robot1,...,robot7
  printf("time(sec)\tdata(volts)\tdistance(mm)\n\r");
  do
  {
    PIDWork++;                     // performance measurement
    time = OS_MsTime();            // 10ms resolution in this OS
    data = OS_Fifo_Get();          // 1000 Hz sampling get from producer
    voltage = (300 * data) / 1024; // in mV
    distance = ADC2millimeter(data);
    printf("%0u.%02u\t%0u.%03u \t%5u\n\r", time / 100, time % 100, voltage / 1000, voltage % 1000, distance);
  } while (time < 200); // change this to mean 2 seconds
  //eFile_EndRedirectToFile();
  ST7735_Message(0, 1, "IR0 (mm) =", distance);
  printf("done.\n\r");
  Name[5] = (Name[5] + 1) & 0xF7; // 0 to 7
  Running = 0;                    // robot no longer running
  OS_Kill();
}

//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void)
{
  if (Running == 0)
  {
    Running = 1;                                // prevents you from starting two robot threads
    NumCreated += OS_AddThread(&Robot, 128, 1); // start a 2 second run
  }
}
//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void)
{
}

//******** Producer ***************
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 10-bit sample
// sends data to the Robot, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned long data)
{
  if (Running)
  {
    if (OS_Fifo_Put(data))
    { // send to Robot
      NumSamples++;
    }
    else
    {
      DataLost++;
    }
  }
}

//******** IdleTask  ***************
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount = 0;
void IdleTask(void)
{
  while (1)
  {
    Idlecount++; // debugging
  }
}

//******** Interpreter **************
// your intepreter from Lab 4
// foreground thread, accepts input from UART port, outputs to UART port
// inputs:  none
// outputs: none
void Interpreter(void)
{
  interpreter_task();
}
// TODO: add the following commands, remove commands that do not make sense anymore
// 1) format
// 2) directory
// 3) print file
// 4) delete file
// execute   eFile_Init();  after periodic interrupts have started

void init_fs_task(void)
{
}

//*******************lab 4 main **********
int realmain(void)
{               // lab 4 real main
  OS_Init();    // initialize, disable interrupts
  Running = 0;  // robot not running
  DataLost = 0; // lost data between producer and consumer
  NumSamples = 0;
  PortD_Init(); // user debugging profile
  f_mount(&g_sFatFs, "", 0);
  //********initialize communication channels
  OS_Fifo_Init(256);
  ADC_Collect(0, 50, &Producer);                // start ADC sampling, channel 0, PE3, 50 Hz
  ADC_Init(3);                                  // sequencer 3, channel 3, PE0, sampling in DAS()
  OS_AddPeriodicThread(&DAS, 10 * TIME_1MS, 1); // 100Hz real time sampling of PE0

  //*******attach background tasks***********
  OS_AddSW1Task(&SW1Push, 2); // PF4, SW1
  OS_AddSW2Task(&SW2Push, 3); // PF0
  OS_InitSemaphore(&doFFT, 0);

  NumCreated = 0;
  // create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter, 128, 2);
  NumCreated += OS_AddThread(&DSP, 128, 3);
  NumCreated += OS_AddThread(&init_fs_task, 128, 1);
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
  CAN_Servo(angle);
  angle += 18;
  if(angle > 180)
  {
    angle = 0;
  }

}

void ControlFollow(int U, int pos)
{
  int TH  = 1000;
  int dir =0;
}

void sensor_task(void)
{
  int i = 0;
  int idx = 0;
  int Front_Left_angle;
  int Front_Right_angle;
  int Left;
  int Right;
  int Front;
  int Error[4];
  for(int i =0;i<4;i++)
    Error[i] = 0xEFDFF1FF; // magic number
  
  int Up=0;
  int Ui = 0;
  int Ud = 0;
  int U=0;
  unsigned long long curtime = 0;;
  unsigned long long prevtime = 0;
  int delta10us = 0;
  int FlagL = 0;
  int FlagR = 0;
  char adc_string[64];
  while(1){

    
    Front_Left_angle = IR_GetData(2) + ANGLELEFT_OFFSET;
    Front_Right_angle = IR_GetData(1) + ANGELRIGHT_OFFSET;
    Left = IR_GetData(3) + LEFT_OFFSET;
    Right = IR_GetData(0) + RIGHT_OFFSET;
   // Front = getdata(Front);
    
    prevtime = curtime;
    curtime = OS_Time();
    for(int i =0;i<4;i++)
    {
      Error[3-i] = Error[3-i-1];
    }
    
    if(Front_Right_angle+Right < Front_Left_angle+Left)
    {
      if(FlagR ==1)
      {
        FlagL = 1;
        FlagR = 0;
        for(int i =0;i<4;i++)
         Error[i] = 0xEFDFF1FF; // magic number
        Ui = 0;
      }
      Error[0] = (1000*Right/Front_Right_angle - (cosTHETA))/10;
    }
    else
    {
      if(FlagL ==1)
      {
        FlagL = 0;
        FlagR = 1;
        for(int i =0;i<4;i++)
         Error[i] = 0xEFDFF1FF; // magic number
        Ui = 0;
      }
      Error[0] = (1000*Left/Front_Left_angle - (cosTHETA))/10;
    }
    
    if(prevtime ==0 || Error[3] == 0xEFDFF1FF)
      continue;
    delta10us = OS_TimeDifference(prevtime,curtime)/800;
    
    Up = KP*Error[0];
    Ui = Ui+ KI*Error[0]*delta10us; // 10us 
    Ud = KD*(Error[0]+3*Error[1]-3*Error[2]-Error[3])/(6*delta10us);
    
    U = Up + Ui + Ud;
    if(Front_Right_angle+Right < Front_Left_angle+Left)
    { 
      ControlFollow(U,0); // 0 is right following
    }
    else
    {
      ControlFollow(U,1); //  is left following
    }
    
    sprintf(adc_string, "Up Ui Ud U %d %d %d %d:  ",  Up,Ui,Ud,U);
    UART_OutString(adc_string);
    UART_OutString("\r\n");
    
  }
}
int Sensor_main(void)
{
  OS_Init(); // initialize, disable interrupts
  CAN0_Open();
  NumCreated = 0;
  NumCreated += OS_AddThread(&Interpreter,128,1);
  NumCreated += OS_AddThread(&sensor_task, 128, 2);
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
  NumCreated += OS_AddThread(&Interpreter, 128, 2);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}

void lcd_task(void)
{
    static int count = 0;
    ST7735_Message(0, 1, "Hello world", count++);
}

int lcd_testmain(void)
{
  OS_Init();
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0xFFFF);
  NumCreated = 0;
  // create initial foreground threads
  NumCreated += OS_AddPeriodicThread(&lcd_task, 1000 * TIME_1MS, 1);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}

// Main stub
int main(void)
{
  return lcd_testmain();
}

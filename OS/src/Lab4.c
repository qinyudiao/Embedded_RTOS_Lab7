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
#include "lidar.h"
#include "LED.h"
#include "DirectionCtrl.h"
#include "PLL.h"

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
#define cosTHETA (525)
// cosTHETA * 1000 value

// Configurable via interpreter
int KP = 35;
int KD = 9;
int KI = 13;

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

#define TIMESLICE 4 * TIME_1MS // thread switch time in system time units
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
//  OS_AddSW1Task(&SW1Push, 2); // PF4, SW1
//  OS_AddSW2Task(&SW2Push, 3); // PF0
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

//void motor_left_turn(void)
//{
//	static int step = 1;
//	static int left_torque = 50;
//	static int right_torque = 50;
//	Motors_SetTorque(left_torque, right_torque);
////	if(step < 25){
////		left_torque = 15;
////	  right_torque = 18;
////		step ++;
////	}
////	else if(step == 25)
////		step = 51;
//	else if(step == 26)
//		step = 1;
//	else if (step > 26){ 
//		left_torque = 15;
//		right_torque = 18;
//		step --;
//	}
		
//}

void motor_task(void)
{
	static int step = 1;
	static int torque = 35;
	// Motors_SetTorque(torque, torque);
	Motors_SetTorque(torque, torque);
	torque += step;
	if((torque > 15) || (torque < -15))
	{
		step = -step;
		torque += step;
	}
	
	//PWM0_0_CMPA_R = 50;
	//PWM0_1_CMPA_R = 50;
	//motor_left_turn();
}

void servo_task(void)
{
  //static int angle = 0;
  Servo_SetAngle(90);
//  angle += 18;
//  if(angle > 180)
//  {
//    angle = 0;
//  }

}

void ControlFollow(int U, int pos)
{
  int TH  = 1000;
  int dir =0;
}

int Up=0;
int Ui = 0;
int Ud = 0;
int U=0;
int Front_Left_angle;
int Front_Right_angle;
int Left;
int Right;
int Front;
int Error[4];
int state;
int LCD_state;  // 0: debug; 1: KP; 2: KD: 3: KI

void sensor_debug_task(void)
{
	while (1) {
  switch (LCD_state)
  {
  case 0:
    ST7735_Message(0, 0, "Up: ", Up);
    ST7735_Message(0, 1, "Ui: ", Ui);
    ST7735_Message(0, 2, "Ud: ", Ud);
    ST7735_Message(0, 3, "U: ", U);
    ST7735_Message(0, 7, "Front: ", Front);
    ST7735_Message(1, 0, "FL Angle: ", Front_Left_angle);
    ST7735_Message(1, 1, "FR Angle: ", Front_Right_angle);
    ST7735_Message(1, 2, "Left: ", Left);
    ST7735_Message(1, 3, "Right: ", Right);
    ST7735_Message(1, 4, "State: ", state);
    break;

  case 1:
    ST7735_Message(0, 0, "KP: ", KP);
    break;
  case 2:
    ST7735_Message(0, 0, "KD: ", KD);
    break;
    case 3:
    ST7735_Message(0, 0, "KI: ", KI);;
    break;
  }
  OS_Sleep(10);
	}
}


//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void)
{
  switch (LCD_state)
  {
  case 0:
	  Error[0] = 0;
    Ui = 0;
    Up = 0;
    Ud = 0;
    U = 0;
    break;
  case 1:
	  KP = (KP >= 35) ? 1 : (KP+1);
    break;
  case 2:
	  KD = (KD >= 50) ? 1 : (KD+2);  
    break;
  case 3:
    KI  = (KI >= 20) ? 1 : (KI+1);
    break;
  }
}

//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void)
{
  switch (LCD_state)
  {
  case 0:
    LCD_state = 1;
	 // ST7735_FillScreen(0xFFFF);
    break;
  case 1:
    LCD_state = 2;
	  //ST7735_FillScreen(0xFFFF);
    break;
    case 2:
    LCD_state = 3;
		//ST7735_FillScreen(0xFFFF);
    break;
    case 3:
    LCD_state = 0; 
	  //ST7735_FillScreen(0xFFFF);
    break;
  }
}

#define SENSOR_TASK_PERIOD (20)  //20 = 50ms
  static int flag_right_back = 0;
  static int flag_left_back = 0;
	static int backTo = 0;
void sensor_task(void)
{
  static int i = 0;
  static int idx = 0;

  static unsigned long long curtime = 0;
  static unsigned long long prevtime = 0;
  static int delta10us = 0;
  static int FlagL = 0;
  static int FlagR = 1;
  static char adc_string[64];
  static int Rightstack[4];
  static int Leftstack[4];
  static int openspaceLeft = 0;
  static int openspaceRight = 0;

  while(1){
    
		if(((OS_Time() / TIME_1MS) / 1000) > 180){
			while(1){
				CAN_MotorTorch(1,1);
        
				OS_Sleep(SENSOR_TASK_PERIOD);
        continue;
			}
		}
			
    Front_Left_angle = lidar_GetData(1) + ANGLELEFT_OFFSET;
    Front_Right_angle = lidar_GetData(0) + ANGELRIGHT_OFFSET;
    Left = IR_GetData(3) + LEFT_OFFSET;
    Right = IR_GetData(2) + RIGHT_OFFSET;
    Front = IR_GetData(0);
    Rightstack[0] = Front_Right_angle;
    Leftstack[0] = Front_Left_angle;
    
    prevtime = curtime;
    curtime = OS_Time();
    for(int i =0;i<3;i++)
    {
      Error[3-i] = Error[3-i-1];
      Leftstack[3-i] = Leftstack[3-i-1];
      Rightstack [3-i]=Rightstack[3-i-1];
    }
    if(Leftstack[3]-Leftstack[2]>800)
      openspaceLeft = 10;
    if(Rightstack[3]-Rightstack[2]>800)
      openspaceRight = 10;
 //   if(Front_Left_angle + Front_Right_angle<150 + 2*ANGLELEFT_OFFSET )
 //     backTo = 15;
    if(FlagR == 1){
      if(Right>250+RIGHT_OFFSET){
        if(Front_Right_angle+Right > Front_Left_angle+Left)
        {
          //LED_RED_ON();
          FlagL =1;
          FlagR=0;
          Ui = Ui/100;
        }
      }
    }else{
      if(Left>250 + LEFT_OFFSET)
      {

        if(Front_Right_angle+Right < Front_Left_angle+Left)
        {
          //LED_RED_OFF();
          FlagL = 0;
          FlagR = 1;
          Ui = Ui/100;
        }
      }
    }
        
    
    if(FlagR == 1)
    {
      Error[0] = (1000*Right/Front_Right_angle - (cosTHETA))/10;
    }
    else
    {
      Error[0] = ((1000*Left)/Front_Left_angle - (cosTHETA))/10;
    }
    
    //if(prevtime ==0 || Error[3] == 0xEFDFF1FF)
    //  continue;
    delta10us = SENSOR_TASK_PERIOD;
 
    Up = KP*Error[0];
    Ui = Ui+ Error[0]*delta10us/KI; // 10us 
    Ud = KD*(Error[0]+3*Error[1]-3*Error[2]-Error[3])/(6*delta10us);
    
    U = Up + Ui + Ud;
    int TH90 = 10000;
    int TH60 = 1000;
    int tmp = 0;
    
    if(backTo > 0)
    {
      backTo--;
      state = 12;
			if(flag_left_back)
				BackLeft();
			else if(flag_right_back)
				BackRight();
			else 
				Back();
      OS_Sleep(SENSOR_TASK_PERIOD);
      continue;
    }
    if(openspaceLeft>0){
      openspaceLeft--;
      if(openspaceLeft<4){
        SlightLeft(250);
        state = 10;
        OS_Sleep(SENSOR_TASK_PERIOD);
        continue;
      }
    }
    
    if(openspaceRight>0){
      openspaceRight--;
      if(openspaceRight<4){
        SlightRight(250);
        state = 11;
        OS_Sleep(SENSOR_TASK_PERIOD);
        continue;
      }
    }
                          
    if(Front_Left_angle<ANGLELEFT_OFFSET+40 || Left<LEFT_OFFSET+40)
    {
        state = 6;
    }
    else if(Front_Right_angle<ANGLELEFT_OFFSET+40||Right<LEFT_OFFSET+40)
    {
        state = 7;
    }

    if(U>10 || U<-10){
      if(FlagR == 1)
      {
         
        if(((100*Front_Right_angle*cosTHETA)/Right)/100>1008) // >90 degree
        {
          if(U<0) {
            tmp = -U/10;
            if(Front_Left_angle > ANGELRIGHT_OFFSET+430)
              tmp = tmp>50? 50:tmp;
            state = 0;
          }
            
            
        }        
        else if(((100*Front_Right_angle*cosTHETA)/Right)/100<992) // <90 degree
        {
          if(U>0) {
            tmp = U/10;
            if(Front_Right_angle > ANGELRIGHT_OFFSET+430)
              tmp = tmp>50? 50:tmp;
            state = 1;
          }
        }
        else
        {
          state = 2;
                              
        }
      }
      else
      {
        if(((100*Front_Left_angle*cosTHETA)/Left)/100<992) // <90 degree
        {
          if(U>0) {
            tmp = U/10;
            if(Front_Left_angle > ANGELRIGHT_OFFSET+430)
              tmp = tmp>50? 50:tmp;
            state = 3;
          }          
        }        
        else if(((100*Front_Left_angle*cosTHETA)/Left)/100>1008) // >90 degree
        {
          if(U<0) {
            tmp = -U/10;
            if(Front_Right_angle > ANGELRIGHT_OFFSET+430)
              tmp = tmp>50? 50:tmp;
            
            state = 4;
          }
        }
        else
        {
            
            state = 5;
                        
        }
      }
    }
    
    if(Front_Left_angle<ANGLELEFT_OFFSET+40 || Left<LEFT_OFFSET+40)
    {
        
        state = 6;
    }
    else if(Front_Right_angle<ANGLELEFT_OFFSET+40||Right<LEFT_OFFSET+40)
    {
        
        state = 7;
    }
    
    if(Front <250)
    {
      if(Front_Left_angle > Front_Right_angle+40)
      {
        
        state = 8;
      }
      else if(Front_Right_angle > Front_Left_angle+40)
      {
        
        state = 9;
      }
    }
    switch(state)
    {
      
      case 0:
        SlightRight(tmp);
        break;
      case 1:
        SlightLeft(tmp);
        break;
      case 2:
        Straight();
        break;
      case 3:
        SlightRight(tmp);
        break;
      case 4:
        SlightLeft(tmp);
        break;
      case 5:
        Straight();
        break;
      case 6:
        SlightRight(80);
        break;
      case 7:
        SlightLeft(80);
        break;
      case 8:
        SlightLeft(350-Front);
        break;
      case 9:
        SlightRight(350-Front);
        break;
    }

        

    OS_Sleep(SENSOR_TASK_PERIOD);
    
  }  
}

void left_bumper_push(void){
//back up left
	flag_left_back = 1;
	backTo = 5;
}
void left_bumper_release(void){
//back up left
	flag_left_back = 0;
	backTo = 10;
}

void right_bumper_push(void){
//back up right
	flag_right_back = 1;
	backTo = 5;
}

void right_bumper_release(void){
//back up right
	flag_right_back = 0;
	backTo = 10;
}

int Sensor_main(void)
{
  
  OS_Init(); // initialize, disable interrupts
  CAN0_Open();
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0xFFFF);
  IR_Init();
  lidar_Init();
  
  
 // LED_Init();  
 // LED_RED_OFF();
  
  NumCreated = 0;
  //NumCreated += OS_AddPeriodicThread(&sensor_task,SENSOR_TASK_PERIOD*TIME_1MS,0);
  NumCreated += OS_AddThread(&sensor_task,128,0);
  NumCreated += OS_AddThread(&sensor_debug_task, 128, 4);
	OS_AddRightBumperTask(&right_bumper_push, &right_bumper_release, 0);
	OS_AddLeftBumperTask(&left_bumper_push, &left_bumper_release, 0);
	
	OS_AddSW1Task(&SW1Push, 0);
  OS_AddSW2Task(&SW2Push, 0);
  LCD_state = 0;

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

unsigned long in, delay, dataRegIn;

void PortC_Init(void)
{
	//SYSCTL_RCGCGPIO_R SYSCTL_RCGC2_R
	SYSCTL_RCGCGPIO_R |= 0x04;           // Port C clock
  delay = SYSCTL_RCGCGPIO_R ;           // wait 3-5 bus cycles
  GPIO_PORTC_DIR_R &= ~0xC0;        // PC6-7 input 
  GPIO_PORTC_AFSEL_R &= ~0xC0;      // not alternative
  GPIO_PORTC_AMSEL_R &= ~0xC0;      // no analog
  GPIO_PORTC_PCTL_R &= ~0xFF000000; // bits for PE1, PE0
  GPIO_PORTC_DEN_R |= 0xC0;         // enable PC6, PC7
	
	//GPIO_PORTC_DATA_R |= 0x20; //start with LED on (PE1 = 1)
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
	
	EnableInterrupts();
	LED_Init();
	PortC_Init();
	
  NumCreated = 0;
  // create initial foreground threads
  NumCreated += OS_AddPeriodicThread(&lcd_task, 1000 * TIME_1MS, 1);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}

void lcd_testtask(void)
{
  static int i=0;
  while(1)
  {
    ST7735_Message(0, 0, "lidars 0: ", lidar_GetData(0));
    ST7735_Message(0, 1, "lidar 1:", lidar_GetData(1));
//    ST7735_Message(0, 2, "lidar 2:", lidar_GetData(2));
//    ST7735_Message(0, 3, "lidar 3:", lidar_GetData(3));

    OS_Sleep(5);
  }
}

int sensor_testmain(void) {
  OS_Init();
	ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0xFFFF);
  LED_Init();
  lidar_Init();
	OS_AddThread(&Interpreter, 128, 2);
	OS_AddThread(&lcd_testtask, 128, 3);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}

int pc6,pc7 =0;
static int count = 0;
void portc_task(void)
{
		//pc6 = (GPIO_PORTC_DATA_R & 0x40) >> 6;
		//pc7 = (GPIO_PORTC_DATA_R & 0x80) >> 7;
    //ST7735_Message(0, 0, "PC6: ", 0);
    //ST7735_Message(0, 1, "PC7 : ", 0);
    ST7735_Message(0, 1, "Hello world", count++);
    //OS_Sleep(5);
}

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

void led_flash_task(void){
    //delay100ms(1); //delay for (1) 100 ms interval
    in = (GPIO_PORTC_DATA_R&0x80)|(GPIO_PORTC_DATA_R&0x40); // in 0 if not pressed, 1 if pressed
		//in = (GPIO_PORTC_DATA_R&0x80); // in 0 if not pressed, 1 if pressed
		//If switch pressed (PE0=1), toggle LED once, else turn LED ON
    //out = (in xor 0x01) << 1 (shift to PE1 LED output)
		//so out = 1 if switch NOT pressed (0 xor 1 = 1 = LED ON)
		//and out = 0 if switch is pressed (1 xor 1 = 0)
		//this works since you can assume LED on all the time if
		//switch not pressed, so to toggle it, just xor it with in = 1
		//to toggle it off when switch pressed
		//did NOT work!  LED did not toggle
    //out = (in^0x01)<<1;   // out 2 if not pressed, 0 if switch pressed
	  //if (in == 0x80) //PE0 = switch = pressed
    if ((in == 0xC0)|(in == 0x40)|(in == 0x80)) //PE0 = switch = pressed
		{
				LED_RED_TOGGLE();
		}
}

int sensor_back_main(void) {
  OS_Init();
	//PLL_Init(Bus80MHz);
	ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0xFFFF);
	EnableInterrupts();
  LED_Init();
  //PortC_Init();
	NumCreated = 0;
	NumCreated += OS_AddPeriodicThread(&portc_task, 1000 * TIME_1MS, 2);
	NumCreated += OS_AddPeriodicThread(&led_flash_task, 1000 * TIME_1MS, 2);
	//OS_AddSW2Task(&right_bumper_push, 1);
	OS_AddSW1Task(&left_bumper_push, 1);
	OS_AddRightBumperTask(&right_bumper_push,&right_bumper_push, 0);
	OS_AddLeftBumperTask(&left_bumper_push,&right_bumper_push, 0);
	//NumCreated += OS_AddPeriodicThread(&led_flash_task, 1000 * TIME_1MS, 2);
  

	
	//LED_RED_TOGGLE();
	
	//OS_AddThread(&portc_task, 128, 2);
	//OS_AddThread(&lcd_testtask, 128, 3);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;
}
// Main stub
int main(void)
{
	return Sensor_main();
 // return Sensor_main();
}

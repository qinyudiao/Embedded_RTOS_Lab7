
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "interpreter.h"
#include "ADC.h"
#include "UART.h"
#include "ff.h"
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "misc_macros.h"
#include "profiler.h"
#include "timeMeasure.h"
#include "loader.h"
#include "can0.h"

#include <stdint.h>
#include "hw_can.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "can.h"
#include "debug.h"
#include "interrupt.h"

#define MAX_LINE_LENGTH (128)

static uint16_t sample_buffer[10];

static const char strtok_delim[] = "\t ";
long sr = 0;




static const ELFSymbol_t exports[] = { { "ST7735_Message", (void*) &ST7735_Message } };
static const ELFEnv_t env = { exports, 1 };

void TEST_OS(void)
{
  
  int k = TEST_OS_Id();
  char adc_string[64];
    sprintf(adc_string,"%d\n",k);
  
  for(int i =0;i<10;i++)
  {
    UART_OutString(". ");
    TEST_OS_Sleep(1000);
  }
  UART_OutString(adc_string);
  sprintf(adc_string,"%ld\n",TEST_OS_Time());
  TEST_OS_Kill();
  
}

void interpreter_task(void)
{
  static char uart_in_buf[1024];
  while (1)
  {
    UART_OutString("> ");
    UART_InString(uart_in_buf, lengthof(uart_in_buf) - 1);
    UART_OutString("\r\n");
    interpreter_cmd(uart_in_buf);
    memset(uart_in_buf, 0, sizeof(uart_in_buf));
    OS_Sleep(100);
  }
}

static void print_event(const event_t *event)
{
  char event_str[80];
  char *event_types[EVENT_NUM_TYPES] = {
      [EVENT_FGTH_START] = "FG START",
      [EVENT_PTH_START] = "PT START",
      [EVENT_PTH_END] = "PT END",
  };
  sprintf(event_str, "Name: %s  Time: %llu cycles  Type: %s\r\n",
          event->name, event->timestamp, event_types[event->type]);
  UART_OutString(event_str);
}


extern int KP;
extern int KD;
extern int KI;
extern int Ui, Ud,Up,U;
extern int Error[4];

void interpreter_cmd(char *cmd_str)
{
  char *cmd, *arg1, *arg2, *arg3, *arg4, *arg5, *arg6;
  cmd = strtok(cmd_str, strtok_delim);
  if (strcmp(cmd, "adc") == 0)
  {
    zeroes(sample_buffer);
    // ADC_Collect(0, 100, sample_buffer, lengthof(sample_buffer));
    char adc_string[64];
    sprintf(adc_string, "ADC: %d, %d  ", sample_buffer[0], sample_buffer[1]);
    UART_OutString(adc_string);
    UART_OutString("\r\n");
  }
  else if (strcmp(cmd, "lcd") == 0)
  {
    char lcd_string[64];
    int count = 0;
    ST7735_FillScreen(0xFFFF); // set screen to white
    for (int device = 0; device < 2; device++)
    {
      for (int line = 0; line < 4; line++)
      {
        sprintf(lcd_string, "Device: %d  Line: %d   ", device, line);
        ST7735_Message(device, line, lcd_string, count++);
      }
    }
  }
  else if (strcmp(cmd, "time") == 0)
  {
    char time[64];
    sprintf(time, "Time: %llu ms\r\n", OS_Time() / TIME_1MS);
    UART_OutString(time);
  }
  else if (strcmp(cmd, "log") == 0)
  {
    Profiler_Foreach(print_event);
  }
  else if (strcmp(cmd, "critical") == 0)
  {
    char time[64];
    int res = getDisablePercent();
    sprintf(time, "critical time: %d percent\r\n", res);
    UART_OutString(time);
  }
  else if (strcmp(cmd, "clear") == 0)
  {
    Profiler_Clear();
    timeMeasureInit();
    timeMeasurestart();
    Error[0] = 0;
    Ui = 0;
    Up = 0;
    Ud = 0;
    U = 0;
  }
  else if(strcmp(cmd, "cat") == 0)
  {
		FIL FP;
    arg1 = strtok(NULL, strtok_delim);
    int open_failed = f_open(&FP,arg1,FA_READ);
    if(open_failed)
    {
      UART_OutString("Failed to open file.\r\n");
    }
    else
    {
      char* read_failed = 0;
      do {
        char c = 0;
        read_failed = f_gets(&c,2,&FP);
        UART_OutChar(c);
      } while(read_failed);
      UART_OutString("\r\n");
    }
    int close_failed = f_close(&FP);
    if(close_failed)
    {
      UART_OutString("Failed to close file.\r\n");
    }
  }
  else if(strcmp(cmd, "rm") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    int del_failed = f_unlink(arg1);
    if(!del_failed)
    {
      UART_OutString("Failed to remove file.\r\n");
    }
    else
    {
      UART_OutString("Removed.\r\n");
    }
  }
  else if(strcmp(cmd, "touch") == 0)
  {
		FIL FP;
    arg1 = strtok(NULL, strtok_delim);
    int create_failed = f_open(&FP,arg1,FA_CREATE_NEW);
    if(create_failed)
    {
      UART_OutString("Failed to create file.\r\n");
    }
    else
    {
      UART_OutString("Created file.\r\n");
    }

  }
  else if(strcmp(cmd, "echo") == 0)
  {
		FIL FP;
    arg1 = strtok(NULL, strtok_delim);
    int open_failed = f_open(&FP,arg1,FA_WRITE);
    if(open_failed)
    {
      UART_OutString("Failed to open file.\r\n");
    }
    else
    {
      arg2 = arg1 + strlen(arg1) + 1; // Don't tokenize to allow spaces
      char *c = arg2;
			f_lseek(&FP,FP.fsize);
      while(*c)
      {
        int write_failed = f_putc(*c++,&FP);
        if(!write_failed)
        {
          UART_OutString("Failed to write to file.\r\n");
          break;
        }
      }
    }
    int close_failed = f_close(&FP);
    if(close_failed)
    {
      UART_OutString("Failed to close file.\r\n");
    }
  }
	else if(strcmp(cmd, "increase") == 0)
	{
		long sr = StartCritical();
		for(int i = 0;i<10000000;i++)
			OS_Time();
		EndCritical(sr);
		UART_OutString("Increase critical.\r\n");
	}
  else if(strcmp(cmd, "load") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    if (exec_elf(arg1, &env) != 0) { 
      UART_OutString("Failed to launch File.\r\n");
    }
    if (exec_elf(arg1, &env) != 0) { 
      UART_OutString("Failed to launch File.\r\n");
    }
  }
  else if(strcmp(cmd, "test") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    TEST_OS_AddThread(&TEST_OS, 128, 1);
  }
  else if(strcmp(cmd, "kp") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    KP = atoi(arg1);
    UART_OutString("KP is ");
    UART_OutUDec(KP);
    UART_OutString("\r\n");
  }
  else if(strcmp(cmd, "ki") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    KI = atoi(arg1);
    UART_OutString("KI is ");
    UART_OutUDec(KI);
    UART_OutString("\r\n");
  }
  else if(strcmp(cmd, "kd") == 0)
  {
    arg1 = strtok(NULL, strtok_delim);
    KD = atoi(arg1);
    UART_OutString("KD is ");
    UART_OutUDec(KD);
    UART_OutString("\r\n");
  }
  else if(strcmp(cmd, "stop") == 0)
  {
    CAN_Motorstop();
  }
  else if(strcmp(cmd, "start") == 0)
  {
    CAN_Motorstart();  
  }
  
}

/**
 * @file
 * 
 * @brief Real Time Operating System for Labs 2 and 3 
 * EE445M/EE380L.12
 * 
 * Some design inspiration came from the textbook
 * "Real-Time Operating Systems for ARM Cortex-M Microcontrollers"
 * by Jonathan Valvano
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "tm4c123gh6pm.h"
#include "OS.h"
#include "misc_macros.h"
#include "priorityqueue.h"
#include "Switch.h"
#include "ST7735.h"
#include "profiler.h"
//#include "timeMeasure.h"
#include "UART.h"
#include "PLL.h"
#include "heap.h"
#include "LED.h"

#define PRIORITY_SCHED 1 // 0 for round robin, 1 for priority scheduler
#define JITTERSIZE 64
unsigned long const JitterSize = JITTERSIZE;
unsigned long JitterHistogram1[JITTERSIZE] = {
    0,
};
unsigned long JitterHistogram2[JITTERSIZE] = {
    0,
};

#define BLOCKING_SEMAS 1 // 0 for spin-waiting semaphores, 1 for blocking semaphores

bool save_ctx_global = true;
tcb_t *tcb_list_head = 0;
tcb_t *tcb_sleep_head = 0;
tcb_t *cur_tcb = 0;
tcb_t *next_tcb = 0;

void pushq(tcb_t *node);
void pop_sema(tcb_t **semahead);
void push_semaq(tcb_t *node, tcb_t **semahead);
uint32_t peek_priority(tcb_t *head);
static void insert_tcb(tcb_t *new_tcb);
static void remove_tcb(tcb_t *tcb, bool free_mem);

#if PRIORITY_SCHED
static void choose_next_with_prio(void)
{
  
  long sr = StartCritical();
  if (cur_tcb->next->priority == tcb_list_head->next->priority)
  {
    next_tcb = cur_tcb->next;
  }
  else
  {
    next_tcb = tcb_list_head->next;
  }

  if (tcb_list_head == 0 || next_tcb == 0)
  {
    // Test point, effectively a conditional breakpoint:
    volatile uint32_t breakpoint = 0;
  }
  EndCritical(sr);
}
#endif

static void ContextSwitch(bool save_ctx)
{
  save_ctx_global = save_ctx;
  NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
  Profiler_Event(EVENT_FGTH_START, cur_tcb->task_name);
}

unsigned int numTasks = 0;
static void IdleTask(void)
{
  while (1)
  {
    // Do maintenance
      //NVIC_ST_CURRENT_R = 0;
      //ContextSwitch(true);
  }
}

static uint64_t get_system_time(void)
{
  unsigned long high = WTIMER0_TBR_R;
  unsigned long low = WTIMER0_TAR_R;
  if (high != WTIMER0_TBR_R)
  {
    high = WTIMER0_TBR_R;
    low = WTIMER0_TAR_R;
  }
  uint64_t systime = high;
  systime <<= 32;
  systime |= low;
  return systime;
}

void SysTick_Handler(void)
{
  ContextSwitch(true);
#if PRIORITY_SCHED
  // Need to make sure we wrap to list head
  // if we reach the end of the set of highest prio tasks
  choose_next_with_prio();
#endif
}

static void TaskReturn(void)
{
  // Kill this task
  OS_Kill();
  //while (1);
}

void OS_Init(void)
{
  //FIRST_DisableInterrupts();
  PLL_Init(Bus80MHz);
  UART_Init();
//   ST7735_InitR(INITR_REDTAB);
//   ST7735_FillScreen(0xFFFF);

  // Activate PendSV interrupt with lowest priority
  NVIC_SYS_PRI3_R |= (7 << 21);
  // Activate Systick interrupt with 2nd lowest priority
  NVIC_SYS_PRI3_R |= ((uint32_t)6 << 29);
  SYSCTL_RCGCTIMER_R |= 1 << 3;                  // activate timer3
  volatile uint32_t __delay = SYSCTL_RCGCGPIO_R; // allow time for clock to stabilize

  TIMER3_CTL_R = 0;           // Disable timer 3 during setup
  TIMER3_CFG_R = 0;           // Set timer 3 to 32-bit counter
  TIMER3_TAMR_R = 0x00000001; // configure for down-count mode
  TIMER3_TAPR_R = 0;          // prescale value for trigger
  TIMER3_TAILR_R = -1;        // start value for trigger
  TIMER3_IMR_R = 0x00000001;  // disable timer 3A time-out interrupt
  TIMER3_CTL_R |= 1<<1;       // Stall timer 3 when CPU halted by debugger
  TIMER3_CTL_R &= ~(1);       // disable timer 3A

  //Enable it when it needs

  // Set up SysTick timer
  NVIC_ST_CURRENT_R = 0;

  // Set up WTIMER0 as 64-bit time source
  SYSCTL_RCGCWTIMER_R |= 1; // activate Wtimer0
  while ((SYSCTL_RCGCWTIMER_R & 1) == 0)
    ;
  WTIMER0_CTL_R &= ~1;         // Disable Wtimer 0 during setup
  WTIMER0_CFG_R = 0;           // Set Wtimer 0 to 64-bit counter, indiv
  WTIMER0_TBMR_R = 0x00000012; // configure for periodic mode, count up
  WTIMER0_TAMR_R = 0x00000012; // configure for periodic mode, count up
  WTIMER0_TBPR_R = 0;          // prescale value for trigger
  WTIMER0_TAPR_R = 0;          // prescale value for trigger
  WTIMER0_CTL_R |= 1 << 1;     // Stall Wtimer0 on CPU debugger halt
  WTIMER0_CTL_R |= 1;          // Kick off Wtimer0


  //timeMeasureInit();

  OS_AddThread(IdleTask, 128, 255);
  Profiler_Init();
  Heap_Init();
}

void OS_InitSemaphore(Sema4Type *semaPt, long value)
{
  semaPt->head = 0;
  semaPt->Value = value;
}

void OS_Wait(Sema4Type *semaPt)
{
#if BLOCKING_SEMAS

  long sr = StartCritical();
  semaPt->Value--;
  if (semaPt->Value < 0)
  {
    remove_tcb(cur_tcb, false);
    // remove from active TCB
    push_semaq(cur_tcb, &semaPt->head);
    NVIC_ST_CURRENT_R = 0; // Make sure next thread gets full time slice
#if PRIORITY_SCHED
    choose_next_with_prio();
#endif
    ContextSwitch(true);
  }
  EndCritical(sr);
#else
  int oldval;

  // read the semaphore value
  oldval = __ldrex(&(semaPt->Value));
  // loop again if it is locked and we are blocking
  // or setting it with strex failed
  while (!(oldval > 0) || __strex(oldval - 1, &(semaPt->Value)) != 0)
  {
    // OS_Suspend();
    oldval = __ldrex(&(semaPt->Value));
  }
#endif
}

void OS_Signal(Sema4Type *semaPt)
{

#if BLOCKING_SEMAS
  long sr = StartCritical();
  semaPt->Value++;
  if (semaPt->Value <= 0)
  {
    bool need_ctx_switch = (semaPt->head->priority < cur_tcb->priority);
    pop_sema(&semaPt->head);
#if PRIORITY_SCHED
    if (need_ctx_switch)
    {
      ContextSwitch(true);
    }
#endif
  }
  EndCritical(sr);
#else

  int oldval;
  do
  {
    oldval = __ldrex(&(semaPt->Value));
  } while (__strex(oldval + 1, &(semaPt->Value)) != 0);
#endif
}

void OS_bWait(Sema4Type *semaPt)
{
#if BLOCKING_SEMAS
  long sr = StartCritical();
  semaPt->Value--;
  if (semaPt->Value < 0)
  {
    remove_tcb(cur_tcb, false);
    // remove from active TCB
    push_semaq(cur_tcb, &semaPt->head);
    NVIC_ST_CURRENT_R = 0; // Make sure next thread gets full time slice
		
#if PRIORITY_SCHED
    choose_next_with_prio();
#endif
    ContextSwitch(true);
  }
  EndCritical(sr);
#else

  int oldval;

  // read the semaphore value
  oldval = __ldrex(&(semaPt->Value));
  // loop again if it is locked and we are blocking
  // or setting it with strex failed
  while (!(oldval > 0) || __strex(oldval & 0, &(semaPt->Value)) != 0)
  {
    OS_Suspend();
    oldval = __ldrex(&(semaPt->Value));
  }
#endif
}

void OS_bSignal(Sema4Type *semaPt)
{
#if BLOCKING_SEMAS
  long sr = StartCritical();
  if (semaPt->Value < 1)
  {
    semaPt->Value++;
  }
  if (semaPt->Value <= 0)
  {
    bool need_ctx_switch = (semaPt->head->priority < cur_tcb->priority);
    pop_sema(&semaPt->head);
#if PRIORITY_SCHED
    if (need_ctx_switch)
    {
      ContextSwitch(true);
    }
#endif
  }
  EndCritical(sr);
#else
  int oldval;
  do
  {
    oldval = __ldrex(&(semaPt->Value));
  } while (__strex(oldval | 1, &(semaPt->Value)) != 0);
#endif
}

static void remove_tcb(tcb_t *tcb, bool free_mem)
{
  long sr = StartCritical();
  if(tcb_list_head == 0)
  {
    // Empty list
    EndCritical(sr);
    return;
  }
  if (tcb_list_head->next == tcb_list_head)
  {
    // One task in list
    if (free_mem)
    {
      // Free this TCB, return to pool
      tcb_list_head->magic = 0;
    }
    tcb_list_head = 0;
    EndCritical(sr);
    return;
  }
  tcb_t *iter = tcb_list_head;
  do
  {
    // Until we wrap around to head
    if (iter->next == tcb)
    {
      iter->next = tcb->next;
      if (free_mem)
        tcb->magic = 0; // Free this TCB, return to pool
			tcb->next = 0;
			EndCritical(sr);
      return;
    }
    iter = iter->next;
  } while (iter != tcb_list_head);
  EndCritical(sr);
}

static void insert_tcb(tcb_t *new_tcb) // priority insert
{
  long sr = StartCritical();
	new_tcb->next = 0;
  if (tcb_list_head == 0)
  {
    tcb_list_head = new_tcb;
    tcb_list_head->next = tcb_list_head;
  }
  else
  {
		tcb_t *tmp = tcb_list_head->next;
		tcb_t *prev = tcb_list_head;
#if PRIORITY_SCHED
    // Maintain priority order
    while (tmp != tcb_list_head && tmp->priority <= new_tcb->priority)
    {
			prev = prev->next;
      tmp = tmp->next;
      if(tmp == prev && tcb_list_head!= prev)
      {
        prev->next = tcb_list_head;
        break;
      }
       
    }
    if(tmp == new_tcb){
      int a = 0;}
    new_tcb->next = tmp;
    prev->next = new_tcb;
#else
    new_tcb->next = tcb_list_head->next;
    tcb_list_head->next = new_tcb;
#endif
  }
	
  EndCritical(sr);
}

#define MAX_TASKS (7)
#define MAX_STACK_DWORDS (128) // 256 * 10tasks * 8Bytes = ~20K

static int next_id = 0;
static tcb_t tcb_pool[MAX_TASKS];
static tcb_t checkpoint_tcb;
static long long *checkpoint_stack = (long long *)0x20007600;
// Stacks need to be dword aligned
static long long stack_pool[MAX_TASKS][MAX_STACK_DWORDS];

int __OS_AddThread(void (*task)(void),
                   unsigned long stackSize,
                   unsigned long priority,
                   char *task_name,
                   pcb_t *parent_process)
{
  unsigned long stackDWords = stackSize;
    //(stackSize / 8) + (stackSize % 8 == 0 ? 0 : 1); // Round up integer div
  tcb_t *tcb = 0;

  long sr = StartCritical();
  // Validate input
  if (task == 0)
  {
    // NULL func pointer
    EndCritical(sr);
    return 0; // Fail
  }
  if (priority > 5 && task != IdleTask)
  {
    // Invalid priority
    EndCritical(sr);
    return 0; // Fail
  }
  if (stackSize < 56)
  {
    // Stack too small to fit initial state
    EndCritical(sr);
    return 0; // Fail
  }
  if (numTasks >= MAX_TASKS)
  {
    // Ran out of TCBs
    EndCritical(sr);
    return 0;
  }
  if (stackDWords > MAX_STACK_DWORDS)
  {
    // Stack requested too large
    EndCritical(sr);
    return 0;
  }

  for (unsigned int i = 0; i < MAX_TASKS; i++)
  {
    if (tcb_pool[i].magic != TCB_MAGIC)
    {
      if(strcmp(task_name,"&checkpoint_entire" )!=0 && strcmp(task_name,"&restart_entire" )!=0){
        tcb = &tcb_pool[i];
        tcb->sp = (long *)&stack_pool[i][stackDWords];
        break;
      }
      else
      {
        tcb = &checkpoint_tcb;
        tcb->sp = (long*)checkpoint_stack;
        break;
      }
    }
  }
  if (tcb == 0)
  {
    // Didn't find a free TCB for some reason
    EndCritical(sr);
    return 0;
  }
  for (int i =0;i<2*stackDWords;i++)
  {
    *(tcb->sp-i-1) = 0xFAFA0000|i;
  }
  *(--tcb->sp) = 0x01000000L;      // xPSR, with Thumb state enabled
  *(--tcb->sp) = (long)task;       // PC
  *(--tcb->sp) = (long)TaskReturn; // LR
  *(--tcb->sp) = 0x12121212;       // R12
  *(--tcb->sp) = 0x03030303;       // R3
  *(--tcb->sp) = 0x02020202;       // R2
  *(--tcb->sp) = 0x01010101;       // R1
  *(--tcb->sp) = 0x00000000;       // R0
  *(--tcb->sp) = 0x11111111;       // R11
  *(--tcb->sp) = 0x10101010;       // R10
  *(--tcb->sp) = (parent_process) ? (long)parent_process->data:(long)(tcb->sp - 6);       // R9
  *(--tcb->sp) = 0x08080808;       // R8
  *(--tcb->sp) = 0x07070707;       // R7
  *(--tcb->sp) = 0x06060606;       // R6
  *(--tcb->sp) = 0x05050505;       // R5
  *(--tcb->sp) = 0x04040404;       // R4

  tcb->id = next_id++;
  tcb->priority = priority;
  tcb->period = 0; // 0 = aperiodic
  tcb->wake_time = 0;
  tcb->next = 0;
  tcb->magic = TCB_MAGIC; // Write magic to mark valid
  tcb->task = task;
  tcb->task_name=  task_name;
  tcb->parent_process = parent_process;
  if(tcb->parent_process)
  {
    tcb->parent_process->num_threads++;
  }
  
  insert_tcb(tcb);
  numTasks++;
  EndCritical(sr);
  return 1; // Success
}

unsigned long OS_Id(void)
{
  return cur_tcb->id;
}

unsigned long MaxJitter = 0;

void Jitter(void)
{

  ST7735_Message(0, 2, "Max Jitter 0.1us=", MaxJitter);
}

static void JitterGet(unsigned long cur_time, unsigned long PERIOD, int timer)
{
  static unsigned long LastTime1 = 0; // time at previous ADC sample
  static unsigned long LastTime2 = 0; // time at previous ADC sample

  long jitter; // time between measured and expected, in us

  unsigned long *last_time;
  unsigned long *histogram;

  if (timer == 1)
  {
    last_time = &LastTime1;
    histogram = JitterHistogram1;
  }
  else
  {
    last_time = &LastTime2;
    histogram = JitterHistogram2;
  }

  unsigned long diff = OS_TimeDifference(*last_time, cur_time);

  if (*last_time != 0)
  {
    if (diff > PERIOD)
    {
      jitter = (diff - PERIOD + 4) / 8; // in 0.1 usec
    }
    else
    {
      jitter = (PERIOD - diff + 4) / 8; // in 0.1 usec
    }

    if (jitter > MaxJitter)
    {
      MaxJitter = jitter; // in usec
    }                     // jitter should be 0

    if (jitter >= JitterSize)
    {
      jitter = JITTERSIZE - 1;
    }

    histogram[jitter]++;
  }

  *last_time = cur_time;
}

static int numPeriodicTasks = 0;
static void (*WTimer1ATask)(void) = 0;
static void (*WTimer1BTask)(void) = 0;
static char *WTimer1ATask_name = 0;
static char *WTimer1BTask_name = 0;

void WideTimer1A_Handler(void)
{
  WTIMER1_ICR_R = 1;    // Clear Wtimer 1A timeout interrupt
  NVIC_ST_CTRL_R &= ~1; // Stop systick
  if (WTimer1ATask)
  {
    JitterGet(OS_Time(), WTIMER1_TAILR_R, 1);
    Profiler_Event(EVENT_PTH_START, WTimer1ATask_name);
    WTimer1ATask();
    Profiler_Event(EVENT_PTH_END, WTimer1ATask_name);
  }
  NVIC_ST_CTRL_R |= 1; // Stop systick
}

void WideTimer1B_Handler(void)
{
  WTIMER1_ICR_R = 1 << 8; // Clear Wtimer 1B timeout interrupt
  NVIC_ST_CTRL_R &= ~1;   // Stop systick
  if (WTimer1BTask)
  {
    JitterGet(OS_Time(), WTIMER1_TBILR_R, 2);
    Profiler_Event(EVENT_PTH_START, WTimer1BTask_name);
    WTimer1BTask();
    Profiler_Event(EVENT_PTH_END, WTimer1BTask_name);
  }
  NVIC_ST_CTRL_R |= 1; // Stop systick
}

int OS_AddPeriodicThread_priv(void (*task)(void),
                              unsigned long period,
                              unsigned long priority,
                              char *task_name)
{
  long sr = StartCritical();
  if (numPeriodicTasks == 0)
  {
    SYSCTL_RCGCWTIMER_R |= 1 << 1; // activate Wtimer1
    while ((SYSCTL_RCGCWTIMER_R & (1 << 1)) == 0)
      ;

    // Use WTIMER 1B
    WTIMER1_CTL_R &= ~(1 << 8);   // Disable Wtimer 1B during setup
    WTIMER1_CFG_R = 4;            // Set Wtimer 1 to 32-bit counter, indiv
    WTIMER1_TBMR_R = 0x00000002;  // configure for periodic mode, default down-count settings
    WTIMER1_TBPR_R = 0;           // prescale value for trigger
    WTIMER1_TBILR_R = period - 1; // start value for trigger
    WTIMER1_IMR_R |= 1 << 8;      // enable Wtimer 1B time-out interrupt
    WTIMER1_CTL_R |= 1 << 9;      // Stall Wtimer 1B on CPU debugger halt
    WTimer1BTask = task;
    WTimer1BTask_name = task_name;

    // Set up interrupt controller, interrupt 97
    NVIC_PRI24_R = (NVIC_PRI24_R & 0xFFFF1FFF) | (priority << 13);

    // Kick off Wtimer
    WTIMER1_CTL_R |= 1 << 8; // enable Wtimer1B 32-b, periodic, no interrupts
  }
  else if (numPeriodicTasks == 1)
  {

    // Use WTIMER 1A
    WTIMER1_CTL_R &= ~1; // Disable Wtimer 1A during setup
    // WTIMER1_CFG_R = 0;            // Set Wtimer 1 to 32-bit counter, indiv
    WTIMER1_TAMR_R = 0x00000002;  // configure for periodic mode, default down-count settings
    WTIMER1_TAPR_R = 0;           // prescale value for trigger
    WTIMER1_TAILR_R = period - 1; // start value for trigger
    WTIMER1_IMR_R |= 1;           // enable Wtimer 1A time-out interrupt
    WTIMER1_CTL_R |= 1 << 1;      // Stall Wtimer 1A on CPU debugger halt
    WTimer1ATask = task;
    WTimer1ATask_name = task_name;

    // Set up interrupt controller, interrupt 96
    NVIC_PRI24_R = (NVIC_PRI24_R & 0xFFFFFF1F) | (priority << 5);

    // Kick off Wtimer
    WTIMER1_CTL_R |= 1; // enable Wtimer1A 32-b, periodic, no interrupts
  }
  else
  {
    // Can't support more than 2 periodic tasks
    EndCritical(sr);
    return 0;
  }
  numPeriodicTasks++;
  EndCritical(sr);
  return 1;
}

int OS_AddSW1Task(void (*task)(void), unsigned long priority)
{
  Switch_Init(task, 0);

  return 0;
}

int OS_AddSW2Task(void (*task)(void), unsigned long priority)
{
  Switch2_Init(task, 0);
  return 0;
}

void OS_Sleep(unsigned long sleepTime)
{
  long sr = StartCritical();
  sleepTime *= TIME_1MS;
  cur_tcb->wake_time = ((sleepTime));
  remove_tcb(cur_tcb, false);
  // remove from active TCB //
  pushq(cur_tcb);
  NVIC_ST_CURRENT_R = 0; // Make sure next thread gets full time slice
  #if PRIORITY_SCHED
  // Need to make sure we wrap to list head
  // if we reach the end of the set of highest prio tasks
  choose_next_with_prio();
  #endif
  ContextSwitch(true);
  EndCritical(sr);
}

void OS_Kill(void)
{
  long sr = StartCritical();
  //  tcb_t *next_tcb = cur_tcb->next;
  remove_tcb(cur_tcb, true);
  if(cur_tcb->parent_process && (--(cur_tcb->parent_process->num_threads) <= 0))
  {
    // Last thread, free process memory
    Heap_Free(cur_tcb->parent_process->text);
    Heap_Free(cur_tcb->parent_process->data);
    Heap_Free(cur_tcb->parent_process);
  }
  cur_tcb = next_tcb;
  next_tcb = cur_tcb->next;
  numTasks--;
  NVIC_ST_CURRENT_R = 0; // Make sure next thread gets full time slice
  ContextSwitch(false);
  EndCritical(sr);
  while(1);
}

void OS_Suspend(void)
{
  ContextSwitch(true);
}

#define OSFIFOSIZE 32            // Must be power of two
static uint32_t volatile OSPutI; // put next
static uint32_t volatile OSGetI; // get next

unsigned long static OSFifo[OSFIFOSIZE];

static Sema4Type fifo_wr_mutex;
static Sema4Type fifo_rd_mutex;
static Sema4Type fifo_level;

void OS_Fifo_Init(unsigned long size)
{
  long sr;
  sr = StartCritical(); // make atomic
  OSPutI = OSGetI = 0;  // Empty
  OS_InitSemaphore(&fifo_wr_mutex, 1);
  OS_InitSemaphore(&fifo_rd_mutex, 1);
  OS_InitSemaphore(&fifo_level, 0);
  EndCritical(sr);
}

int OS_Fifo_Put(unsigned long data)
{
  int ret = 0;
  OS_Wait(&fifo_wr_mutex);
  if ((OSPutI + 1) % OSFIFOSIZE == OSGetI)
  {
    ret = (0); // Failed, fifo full
  }
  else
  {
    OSFifo[OSPutI] = data;              // put
    OSPutI = (OSPutI + 1) % OSFIFOSIZE; // Success, update
    ret = (1);
    OS_Signal(&fifo_level); // Signal 1 more queue item
  }
  OS_Signal(&fifo_wr_mutex);
  return ret;
}

unsigned long OS_Fifo_Get(void)
{
  unsigned long result = 0;
  OS_Wait(&fifo_level); // Wait for 1+ items to be in queue
  OS_Wait(&fifo_rd_mutex);
  result = OSFifo[OSGetI];
  OSGetI = (OSGetI + 1) % OSFIFOSIZE; // Success, update
  OS_Signal(&fifo_rd_mutex);
  return result;
}

long OS_Fifo_Size(void)
{
  return ((uint32_t)(OSPutI - OSGetI));
}

Sema4Type mailboxFull;
Sema4Type mailboxMutex;
unsigned long mailBox;

void OS_MailBox_Init(void)
{
  mailBox = 0;
  OS_InitSemaphore(&mailboxFull, 0);
  OS_InitSemaphore(&mailboxMutex, 1);
}

void OS_MailBox_Send(unsigned long data)
{
  OS_Wait(&mailboxMutex);
  mailBox = data;
  OS_Signal(&mailboxFull);
  OS_Signal(&mailboxMutex);
}

unsigned long OS_MailBox_Recv(void)
{
  unsigned long data;
  OS_Wait(&mailboxFull);
  OS_Wait(&mailboxMutex);
  data = mailBox;
  OS_Signal(&mailboxMutex);
  return data;
}

unsigned long long OS_Time(void)
{
  return get_system_time();
}

unsigned long long OS_TimeDifference(unsigned long long start, unsigned long long stop)
{
  return stop - start;
}

static uint64_t time_when_cleared = 0;
void OS_ClearMsTime(void)
{
  time_when_cleared = get_system_time();
}

unsigned long OS_MsTime(void)
{
  uint64_t timediff = (get_system_time() - time_when_cleared);
  timediff = timediff / TIME_1MS;
  return (unsigned long)timediff;
}

void OS_Launch(unsigned long theTimeSlice)
{
  // Check if we can launch
  if (tcb_list_head == 0)
  {
    return;
  }
  cur_tcb = tcb_list_head;
  next_tcb = cur_tcb->next;
  NVIC_ST_RELOAD_R = theTimeSlice;
  NVIC_ST_CTRL_R |= 0x7;

  NVIC_EN1_R |= 1 << (35 - 32); // 9) enable IRQ 35 in NVIC
  NVIC_EN3_R |= 1 << 0;        // Enable wtimer1A interrupt
  NVIC_EN3_R |= 1 << 1;        // Enable wtimerB interrupt

  timeMeasurestart();
  ContextSwitch(false);
  EnableInterrupts();
}

uint32_t peek_waketime(tcb_t *head)
{
  return head->wake_time;
}

uint32_t peek_priority(tcb_t *head)
{
  return head->priority;
}

void pop()
{
  long sr = StartCritical();

  tcb_t *head = tcb_sleep_head;
  tcb_t *tmpinq = head;
  if (head == 0)
  {
    EndCritical(sr);
    return;
  }

  tmpinq = head->next;
  insert_tcb(head);

  head->wake_time = 0;
  tcb_sleep_head = tmpinq;
  head = tmpinq;

  if (head != 0)
  {
    TIMER3_CTL_R &= ~(1); // disable timer 3
    TIMER3_TAILR_R = head->wake_time;
    TIMER3_TAV_R = head->wake_time;
    TIMER3_CTL_R |= 0x1;
  }
	#if PRIORITY_SCHED
  choose_next_with_prio();
#endif
  EndCritical(sr);
}

void pushq(tcb_t *node)
{
  // outside is already disinterrupted
  tcb_t *start = tcb_sleep_head;
  TIMER3_CTL_R &= ~(1); // disable timer 3
  uint32_t cur_timer3 = TIMER3_TAR_R;
  node->next = 0;
  if (tcb_sleep_head == 0)
  {
    TIMER3_TAILR_R = node->wake_time;
    TIMER3_TAV_R = node->wake_time;
    tcb_sleep_head = node;
  }
  else if (node->wake_time < cur_timer3)
  {
    node->next = tcb_sleep_head;
    tcb_sleep_head = node;
    node->next->wake_time = cur_timer3 - node->wake_time;
    if(node->next->wake_time < 80)
    {
      node->next->wake_time = 80;
    }
    TIMER3_TAV_R = node->wake_time;
    TIMER3_TAILR_R = node->wake_time;
    // NVIC_UNPEND1_R = 1 << (35 - 32); // if before this statement end tick could end, then remove.
  }
  else
  {
    start->wake_time = cur_timer3; // Make up to date
    node->wake_time -= start->wake_time;
    while (start->next != 0 && start->next->wake_time <= node->wake_time)
    {
      node->wake_time = node->wake_time - start->next->wake_time;
      start = start->next;
    }
    if(node->wake_time < 80)
    {
      node->wake_time = 80;
    }
    node->next = start->next;
    start->next = node;
    if(node->next)
    {
      node->next->wake_time -= node->wake_time;
      if(node->next->wake_time < 80)
      {
        node->next->wake_time = 80;
      }
    }
  }
  TIMER3_CTL_R |= 0x1; // enable timer 3
}

void Timer3A_Handler()
{
  pop();
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;
}

void pop_sema(tcb_t **semahead)
{
  long sr = StartCritical();
  tcb_t *next_in_wait = 0;

  tcb_t *head = *semahead;
  if (head == 0)
  {
    EndCritical(sr);
    return;
  }

  next_in_wait = head->next;
  insert_tcb(head);

#if PRIORITY_SCHED
  // Need to update next_tcb in case
  // popped task is higher prio than current.
  choose_next_with_prio();
#endif
  *semahead = next_in_wait;
  EndCritical(sr);
}

void push_semaq(tcb_t *node, tcb_t **semahead)
{
  // outside is already disinterrupted && set node

	
  node->next = 0;
  if (*semahead == 0)
  {
    *semahead = node;
  }
  else if(node->priority < (*semahead)->priority)
	{
		tcb_t *tmp = *semahead;
		*semahead = node;
		node->next = tmp;
	}
	else
  {
		tcb_t *prev = *semahead;
		tcb_t *start = (*semahead)->next;
	
    while (start != 0 && start->priority <= node->priority)
    {
			prev = prev->next;
      start = start->next;
    }
    prev->next = node;
    node->next = start;
  }
}


int OS_AddProcess(void(*entry)(void),void *text, void *data, unsigned long stackSize, unsigned long priority)
{
  pcb_t *new_process = (pcb_t *)Heap_Malloc(sizeof(pcb_t));
  memset(new_process, 0, sizeof(new_process));
  new_process->data = data;
  new_process->text = text;
  if(__OS_AddThread(entry, stackSize, priority, "Process main", new_process) == 0)
  {
    return -1;
  }
  tcb_t *tmp = cur_tcb;
  while(tmp->parent_process == new_process)
  {
    tmp = tmp->next;
  }

	return 0;
}

int C_SVC_handler(unsigned int number, unsigned int *reg)
{
	switch(number)
	{
		case 0:
			return OS_Id();
		case 1:
		  OS_Kill();
			return reg[0];
		case 2:
			OS_Sleep(reg[0]);
			return reg[0];
		case 3:
			return OS_Time();
		case 4:
			return OS_AddThread((void *)reg[0],reg[1],reg[2]);
		
	}
	return 0;
}


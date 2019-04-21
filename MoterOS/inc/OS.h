/**
 * @file
 * 
 * @brief Real Time Operating System for Labs 2 and 3 
 * EE445M/EE380L.12
 * 
 * RTOS kernel capable of round-robin scheduling, up to 2 low-jitter
 * periodic tasks.
 * 
 * Reserves WTIMER1A and B for periodic task scheduling.
 * Reserves SysTick timer for round-robin scheduling.
 * Reserves WTIMER0 as a 64-bit time source.
 * 
 * Interface by Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
 * Implementation by Riley Wood and Jeageun Jung
 * @author Riley Wood and Jeageun Jung
 */

#ifndef OS_H
#define OS_H

#include <stdint.h>

// edit these depending on your clock
#define TIME_1MS 80000
#define TIME_2MS (2 * TIME_1MS)
#define TIME_500US (TIME_1MS / 2)
#define TIME_250US (TIME_1MS / 4)

#define TASK_STACK_SIZE 128

#define TCB_MAGIC (0x900d900d)

typedef struct _pcb_s
{
  unsigned long num_threads;
  void *text;
  void *data;
} pcb_t;

typedef struct _tcb_s
{
  long *sp;
  struct _tcb_s *next;
  uint32_t wake_time;
  unsigned long id;
  uint8_t priority;
  uint32_t period; // 0 = aperiodic
  //! magic field must contain TCB_MAGIC for TCB to be valid
  unsigned long magic;
  void (*task)(void);
  char * task_name;
  pcb_t *parent_process;
} tcb_t;


// feel free to change the type of semaphore, there are lots of good solutions
struct Sema4
{
  long Value; // >0 means free, otherwise means busy
	struct _tcb_s   *head;
  // add other components here, if necessary to implement blocking
};
typedef struct Sema4 Sema4Type;

/**
 * initialize operating system, disable interrupts until OS_Launch
 * initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
 */
void OS_Init(void);

/**
 * initialize semaphore 
 * @param semaPt pointer to a semaphore
 */
void OS_InitSemaphore(Sema4Type *semaPt, long value);

/**
 * decrement semaphore 
 * Lab2 spinlock
 * Lab3 block if less than zero
 * @param semaPt pointer to a counting semaphore
 */
void OS_Wait(Sema4Type *semaPt);

/**
 * increment semaphore 
 * Lab2 spinlock
 * Lab3 wakeup blocked thread if appropriate 
 * @param semaPt pointer to a counting semaphore
 */
void OS_Signal(Sema4Type *semaPt);

/**
 * Lab2 spinlock, set to 0
 * Lab3 block if less than zero
 * @param semaPt pointer to a binary semaphore
 */
void OS_bWait(Sema4Type *semaPt);

/**
 * Lab2 spinlock, set to 1
 * Lab3 wakeup blocked thread if appropriate 
 * @param semaPt pointer to a binary semaphore
 */
void OS_bSignal(Sema4Type *semaPt);

/**
 * @brief Print the max periodic task jitter
 *        measured thus far to the ST7735 display.
 */
void Jitter(void);

int __OS_AddThread(void (*task)(void),
                   unsigned long stackSize,
                   unsigned long priority,
                   char *task_name,
                   pcb_t *parent_process);

extern tcb_t *cur_tcb;

/**
 * add a foregound thread to the scheduler
 * stack size must be divisable by 8 (aligned to double word boundary)
 * In Lab 2, you can ignore both the stackSize and priority fields
 * In Lab 3, you can ignore the stackSize fields
 * @param task Task function
 * @param stackSize Size of the stack in bytes. Should be divisible by 8
 * @param priority Priority of the task. 0 is highest, 5 is lowest.
 * @return 1 if successful, 0 if this thread can not be added
 */
#define OS_AddThread(task, stackSize, priority) __OS_AddThread(task,\
                                                               stackSize,\
                                                               priority,\
                                                               #task,\
                                                               cur_tcb ? cur_tcb->parent_process : 0)



/**
 * returns the thread ID for the currently running thread
 * @return Thread ID, number greater than zero 
 */
unsigned long OS_Id(void);


int OS_AddPeriodicThread_priv(void (*task)(void),
                         unsigned long period,
                         unsigned long priority,
                         char *task_name);

/**
 * Add a background periodic task.
 * Typically this function receives the highest priority
 * You are free to select the time resolution for this function
 * It is assumed that the user task will run to completion and return
 * This task can not spin, block, loop, sleep, or kill
 * This task can call OS_Signal  OS_bSignal	 OS_AddThread
 * This task does not have a Thread ID
 * In lab 2, this command will be called 0 or 1 times
 * In lab 2, the priority field can be ignored
 * In lab 3, this command will be called 0 1 or 2 times
 * In lab 3, there will be up to four background threads, and this priority field 
 *           determines the relative priority of these four threads
 * @param task pointer to a void/void background function
 * @param period given in system time units (12.5ns)
 * @param priority 0 is the highest, 5 is the lowest
 * @return 1 if successful, 0 if this thread can not be added
 */
#define OS_AddPeriodicThread(task, period, priority) OS_AddPeriodicThread_priv(task, period, priority, #task)

/**
 * add a background task to run whenever the SW1 (PF4) button is pushed
 * @param pointer to a void/void background function
 * @param priority 0 is the highest, 5 is the lowest
 * @return 1 if successful, 0 if this thread can not be added
 * It is assumed that the user task will run to completion and return
 * This task can not spin, block, loop, sleep, or kill
 * This task can call OS_Signal  OS_bSignal	 OS_AddThread
 * This task does not have a Thread ID
 * In labs 2 and 3, this command will be called 0 or 1 times
 * In lab 2, the priority field can be ignored
 * In lab 3, there will be up to four background threads, and this priority field 
 *           determines the relative priority of these four threads
 */
int OS_AddSW1Task(void (*task)(void), unsigned long priority);

/**
 * add a background task to run whenever the SW2 (PF0) button is pushed
 * @param pointer to a void/void background function
 * @param priority 0 is highest, 5 is lowest
 * @return 1 if successful, 0 if this thread can not be added
 * It is assumed user task will run to completion and return
 * This task can not spin block loop sleep or kill
 * This task can call issue OS_Signal, it can call OS_AddThread
 * This task does not have a Thread ID
 * In lab 2, this function can be ignored
 * In lab 3, this command will be called will be called 0 or 1 times
 * In lab 3, there will be up to four background threads, and this priority field 
 *           determines the relative priority of these four threads
 */
int OS_AddSW2Task(void (*task)(void), unsigned long priority);

/**
 * Place this thread into a dormant state.
 * You are free to select the time resolution for this function.
 * OS_Sleep(0) implements cooperative multitasking.
 * @param sleepTime number of msec to sleep
 */
void OS_Sleep(unsigned long sleepTime);

/**
 * kill the currently running thread, release its TCB and stack
 */
void OS_Kill(void);

/**
 * suspend execution of currently running thread.
 * scheduler will choose another thread to execute.
 * Can be used to implement cooperative multitasking.
 * Same function as OS_Sleep(0).
 */
void OS_Suspend(void);

/**
 * Initialize the Fifo to be empty.
 * In Lab 2, you can ignore the size field.
 * In Lab 3, you should implement the user-defined fifo size.
 * In Lab 3, you can put whatever restrictions you want on size
 *    e.g., 4 to 64 elements
 *    e.g., must be a power of 2,4,8,16,32,64,128
 * @param size Size of the fifo
 * @return none 
 */
void OS_Fifo_Init(unsigned long size);

/**
 * Enter one data sample into the Fifo.
 * Called from the background, so no waiting.
 * Since this is called by interrupt handlers
 * this function can not disable or enable interrupts.
 * @param data Data to put in the FIFO
 * @return true if data is properly saved,
 *         false if data not saved, because it was full
 */
int OS_Fifo_Put(unsigned long data);

/**
 * Remove one data sample from the Fifo.
 * Called in foreground, will spin/block if empty
 * @return data
 */
unsigned long OS_Fifo_Get(void);

/**
 * Check the status of the Fifo.
 * @return returns the number of elements in the Fifo.
 *          Greater than zero if a call to OS_Fifo_Get will return right away,
 *          zero or less than zero if the Fifo is empty,
 *          zero or less than zero if a call to OS_Fifo_Get will spin or block
 */
long OS_Fifo_Size(void);

/**
 * Initialize communication channel
 * @return none
 */
void OS_MailBox_Init(void);

/**
 * Enter mail into the MailBox.
 * This function will be called from a foreground thread.
 * It will spin/block if the MailBox contains data not yet received 
 * @param data to be sent
 * @return none
 */
void OS_MailBox_Send(unsigned long data);

/**
 * Remove mail from the MailBox.
 * This function will be called from a foreground thread.
 * It will spin/block if the MailBox is empty.
 * @return data received
 */
unsigned long OS_MailBox_Recv(void);

/**
 * Return the system time in system time units (12.5ns)
 * @return time in 12.5ns units, 0 to 4294967295
 */
unsigned long long OS_Time(void);

/**
 * Calculates difference between two times.
 * The time resolution should be less than or equal to 1us, and the precision at least 12 bits.
 * It is ok to change the resolution and precision of this function as long as 
 *   this function and OS_Time have the same resolution and precision.
 * @param start Start time measured with OS_Time
 * @param stop Stop time measured with OS_Time
 * @return time difference in 12.5ns units 
 */
unsigned long long OS_TimeDifference(unsigned long long start, unsigned long long stop);

/**
 * Sets the system time to zero (from Lab 1).
 * You are free to change how this works.
 * @return none
 */
void OS_ClearMsTime(void);

/**
 * Reads the current time in msec (from Lab 1).
 * You are free to select the time resolution for this function.
 * It is ok to make the resolution to match the first call to OS_AddPeriodicThread.
 * @return time in ms units
 */
unsigned long OS_MsTime(void);

/**
 * Start the scheduler, enable interrupts.
 * In Lab 2, you can ignore the theTimeSlice field.
 * In Lab 3, you should implement the user-defined TimeSlice field.
 * It is ok to limit the range of theTimeSlice to match the 24-bit SysTick.
 * @param theTimeSlice number of 12.5ns clock cycles for each time slice
 * @return none (does not return)
 */
void OS_Launch(unsigned long theTimeSlice);

/**
 * @brief Launch a process in the OS
 * 
 * @param entry Entry point, usually main() of the process
 * @param text Text (code) section start address
 * @param data Data section start address
 * @param stackSize Size of the stack for the first thread
 * @param priority Priority for the first thread
 * @return int 0 on success, -1 on failure.
 */
int OS_AddProcess(void(*entry)(void),void *text, void *data, unsigned long stackSize, unsigned long priority);
											
extern long StartCritical(void);
extern void EndCritical(long sr);
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);

#endif

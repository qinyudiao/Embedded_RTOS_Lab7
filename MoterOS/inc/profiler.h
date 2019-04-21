/**
 * @file 
 * @author Riley Wood (riley.wood@utexas.edu)
 * @brief Thread profiler utility
 * 
 */

#ifndef _PROFILER_H_
#define _PROFILER_H_

#define EVENT_MAGIC (0x02344629)

#define MAX_EVENTS (100)

typedef enum
{
  EVENT_FGTH_START,
  EVENT_PTH_START,
  EVENT_PTH_END,
  EVENT_NUM_TYPES
} event_type_e;

typedef struct
{
  event_type_e type;
  int magic;
  char * name;
  unsigned long long timestamp;
} event_t;

/**
 * @brief Initialize the thread profiler. Call before use.
 */
void Profiler_Init(void);

/**
 * @brief Register an event has occurred in the profiler.
 * 
 * @param event_id ID of the event that occurred
 *
 * @return -1 on error, 0 on success
 */
int Profiler_Event(event_type_e event_type, char *event_name);

/**
 * @brief Clear profiler history
 */
void Profiler_Clear(void);

/**
 * @brief Executes a function f on each event in the log
 *        in the order they occurred in the system.
 * 
 * @param f Function to execute on each event in the log.
 */
void Profiler_Foreach(void (*f)(const event_t *));


#endif // _PROFILER_H_


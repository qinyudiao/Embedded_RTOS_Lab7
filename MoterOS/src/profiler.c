#include <string.h>
#include "profiler.h"
#include "OS.h"

static event_t elog[MAX_EVENTS];
static unsigned long elog_idx = 0;

void Profiler_Init(void)
{
  Profiler_Clear();
}

int Profiler_Event(event_type_e etype, char *event_name)
{
  unsigned long long timestamp = OS_Time(); // Capture time first thing
  if(elog_idx >= MAX_EVENTS)
  {
    return -1;
  }

  elog[elog_idx].type = etype;
  elog[elog_idx].magic = EVENT_MAGIC;
  elog[elog_idx].name = event_name;
  elog[elog_idx].timestamp = timestamp;

  elog_idx++;
  return 0;
}

void Profiler_Clear(void)
{
  memset(elog, 0, sizeof(elog));
  elog_idx = 0;
}

void Profiler_Foreach(void (*f)(const event_t *))
{
  if(elog_idx < MAX_EVENTS)
  {
    // Log not full, still recording, can't proceed
    return;
  }

  for(int i=0; i<MAX_EVENTS; i++)
  {
    if(elog[i].magic != EVENT_MAGIC)
      break;
    
    f(&elog[i]);
  }
}

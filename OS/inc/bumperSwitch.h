
#include <stdint.h>

extern uint8_t left_bumper_state; // 0:none detected 1:detected
extern uint8_t right_bumper_state;

static void (*left_bumper_task)(void);
static void (*right_bumper_task)(void);

static void left_bumper_debounce(void);
static void right_bumper_debounce(void);
int OS_AddRightBumperTask(void(*task)(void), unsigned long priority);
int OS_AddLeftBumperTask(void(*task)(void), unsigned long priority);

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "motors.h"

#define PB4 (*((volatile unsigned long *)0x40005040))
#define PB7 (*((volatile unsigned long *)0x40005200))
#define PB4_PB7 (*((volatile unsigned long *)0x40005240))

#define MOTOR_PWM_PERIOD (2500000 / 5000) //500 ticks for 5Khz PWM period

static int16_t constrain_duty(int16_t user_input)
{
  if(user_input >= MOTOR_PWM_PERIOD/2)
  {
    user_input = MOTOR_PWM_PERIOD/2 - 1;
  }
  else if(user_input <= -(MOTOR_PWM_PERIOD/2))
  {
    user_input = -(MOTOR_PWM_PERIOD/2 - 1);
  }
  return user_input;
}

void Motors_Init(void)
{
  // Initialize PB5 and PB6 as PWM outputs
  // Initialize PB4 and PB7 as digital outputs
  SYSCTL_RCGCPWM_R |= 0x01;  // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02; // 2) activate port B
  while(!(SYSCTL_RCGCPWM_R & 0x01) || !(SYSCTL_RCGCGPIO_R & 0x02));

  GPIO_PORTB_AFSEL_R &= ~0x90; // disable alt funct on PB4,7
  GPIO_PORTB_AFSEL_R |= 0x60; // enable alt funct on PB5,6
  GPIO_PORTB_PCTL_R &= ~0xFFFF0000; // configure PB5,6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04400000;
  GPIO_PORTB_AMSEL_R &= ~0xF0; // disable analog functionality on PB4,5,6,7
  GPIO_PORTB_DEN_R |= 0xF0; // enable digital I/O on PB4,5,6,7
  GPIO_PORTB_DIR_R |= 0x90;    // make PB4,7 digital outputs

  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; // clear PWM divider field
  SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_32; // configure for /32 divider
  PWM0_0_CTL_R = 0; // disable PWM while initializing
  PWM0_1_CTL_R = 0; // disable PWM while initializing
  // PWM0, Generator A (PWM0/PB6) goes to 0 when count==CMPA counting down and 1 when count==CMPA counting up
  PWM0_0_GENA_R = (PWM_0_GENA_ACTCMPAD_ONE|PWM_0_GENA_ACTCMPAU_ZERO);
  // PWM0, Generator B (PWM3/PB5) goes to 0 when count==CMPA counting down and 1 when count==CMPA counting up
  PWM0_1_GENB_R = (PWM_1_GENB_ACTCMPAD_ONE|PWM_1_GENB_ACTCMPAU_ZERO);
  PWM0_0_LOAD_R = MOTOR_PWM_PERIOD/2; // count from zero to this number and back to zero in (period - 1) cycles
  PWM0_1_LOAD_R = MOTOR_PWM_PERIOD/2; // count from zero to this number and back to zero in (period - 1) cycles
  // Synchronize PWM comparator updates, put in up/down mode, enable generators
  PWM0_ENUPD_R = (PWM_ENUPD_ENUPD0_GSYNC|PWM_ENUPD_ENUPD3_GSYNC);
  PWM0_0_CTL_R = (PWM_0_CTL_CMPAUPD|PWM_0_CTL_MODE|PWM_0_CTL_ENABLE);
  PWM0_1_CTL_R = (PWM_1_CTL_CMPAUPD|PWM_1_CTL_MODE|PWM_1_CTL_ENABLE);
  PWM0_CTL_R = 0x3; // Synchronously update generators 0 and 1
  PWM0_SYNC_R = 0x3; // Reset counters synchronously so they count in-step
  Motors_Brake();
}

static void __set_left(int16_t left_trq)
{
  left_trq = -left_trq;
  left_trq = constrain_duty(left_trq);
  // PB6 (PWM0) and PB7 (digital out)
  if(left_trq > 0)
  {
    PB7 = 1<<7;
    PWM0_0_CMPA_R = (MOTOR_PWM_PERIOD/2 - left_trq);
  }
  else
  {
    PB7 = 0;
    PWM0_0_CMPA_R = -left_trq;
  }
}

static void __set_right(int16_t right_trq)
{
  right_trq = -right_trq;
  right_trq = constrain_duty(right_trq);
  // PB5 (PWM0) and PB4 (digital out)
  if(right_trq > 0)
  {
    PB4 = 1<<4;
    PWM0_1_CMPA_R = (MOTOR_PWM_PERIOD/2 - right_trq);
  }
  else
  {
    PB4 = 0;
    PWM0_1_CMPA_R = -right_trq;
  }
}

static inline void __sync_update(void)
{
  PWM0_CTL_R = 0x3; // Synchronously update generators 0 and 1
}

void Motors_SetTorque(int16_t left_trq, int16_t right_trq)
{
  __set_left(left_trq);
  __set_right(right_trq);
  PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM3EN); // Enable both PWMs
  __sync_update();
}

void Motors_SetTorque_Left(int16_t left_trq)
{
  __set_left(left_trq);
  PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN; // Enable M0PWM0
  __sync_update();
}

void Motors_SetTorque_Right(int16_t right_trq)
{
  __set_right(right_trq);
  PWM0_ENABLE_R |= PWM_ENABLE_PWM3EN; // Enable M0PWM3
  __sync_update();
}

void Motors_Brake(void)
{
  PB4_PB7 = 0;
  PWM0_ENABLE_R &= ~(PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM3EN); // Disable both PWMs
  __sync_update();
}

void Motors_Brake_Left(void)
{
  PB7 = 0;
  PWM0_ENABLE_R &= ~PWM_ENABLE_PWM0EN; // Disable M0PWM0
  __sync_update();
}

void Motors_Brake_Right(void)
{
  PB4 = 0;
  PWM0_ENABLE_R &= ~PWM_ENABLE_PWM3EN; // Disable M0PWM3
  __sync_update();
}

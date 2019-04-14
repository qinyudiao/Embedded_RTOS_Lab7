
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "servo.h"

#define PWM_PERIOD (2500000 / 50) // 2.5MHz PWM clock / 50 Hz PWM period = # ticks

#define DUTY_MIN (PWM_PERIOD / 20) // 2.5% duty = 0.5ms pulse
#define DUTY_MAX (PWM_PERIOD / 8)  // 12.5% duty = 2.5ms pulse

#define DEGREES_MAX (180)

void Servo_Init(void)
{
  // PWM clock = 80MHz PLL clock / 32 = 2.5MHz
  SYSCTL_RCGCPWM_R |= 0x02;  // activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x08; // activate port D
  while (!(SYSCTL_RCGCPWM_R & 0x02) || !(SYSCTL_RCGCGPIO_R & 0x08));

  GPIO_PORTD_AFSEL_R |= 0x01;       // enable alt funct on PD0
  GPIO_PORTD_PCTL_R &= ~0x0000000F; // configure PD0 as M1PWM0
  GPIO_PORTD_PCTL_R |= 0x00000005;
  GPIO_PORTD_AMSEL_R &= ~0x01; // disable analog functionality on PD0
  GPIO_PORTD_DEN_R |= 0x01;    // enable digital I/O on PD0

  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; // clear PWM divider field
  SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_32; // configure for /32 divider
  PWM1_0_CTL_R = 0;                      // disable PWM while initializing
  // Count up mode. PWM goes high on zero, low on compare value
  PWM1_0_GENA_R = (PWM_0_GENA_ACTZERO_ONE | PWM_0_GENA_ACTCMPAU_ZERO);
  PWM1_0_LOAD_R = PWM_PERIOD; // count from zero to this number and back to zero in (period - 1) cycles
  // Synchronize PWM enable/disable to counter, enable generator
  PWM1_ENUPD_R = PWM_ENUPD_ENUPD0_LSYNC;
  PWM1_0_CTL_R = PWM_0_CTL_ENABLE;
  Servo_Off();
}

/**
 * @brief Map degrees (0-180) to duty cycle
 * 0 degrees = 0.5 ms pulse / 20ms period = 2.5% duty
 * 180 degrees = 2.5 ms pulse / 20 ms period = 12.5% duty
 * 
 * @param degrees Angle in degrees
 * @return uint16_t duty cycle as PWM comparator value
 */
static uint16_t degrees_to_duty(uint16_t degrees)
{
  if (degrees > DEGREES_MAX)
  {
    degrees = DEGREES_MAX;
  }

  // Map degrees into range of duty cycles
  return degrees * (DUTY_MAX - DUTY_MIN) / DEGREES_MAX + DUTY_MIN;
}

void Servo_SetAngle(uint16_t degrees)
{
  uint16_t duty = degrees_to_duty(degrees);
  PWM1_0_CMPA_R = duty;
  PWM1_ENABLE_R |= PWM_ENABLE_PWM0EN;
}

void Servo_Off(void)
{
  PWM1_ENABLE_R &= ~PWM_ENABLE_PWM0EN;
}

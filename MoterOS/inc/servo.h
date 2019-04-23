/**
 * @file
 * @author Riley Wood (riley.wood@utexas.edu)
 * @brief Implements a driver for a single hitec HS-300 servo motor.
 * PWM Module 1 is allocated to this driver.
 * 
 * @copyright Copyright (c) 2019
 */

#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdint.h>

/**
 * @brief Initialize the servo module.
 */
void Servo_Init(void);

/**
 * @brief Command the servo to hold a specific angular position.
 * 
 * @param degrees Desired position in degrees (0-180)
 */
void Servo_SetAngle(uint16_t degrees);

/**
 * @brief Stop driving the servo, allowing it to move freely.
 * Call Servo_SetAngle to activate the servo again.
 */
void Servo_Off(void);

#endif // _SERVO_H_

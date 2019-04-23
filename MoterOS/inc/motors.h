/**
 * @file
 * @author Riley Wood (riley.wood@utexas.edu)
 *
 * @brief Interface to two DC motors controlled by PWM. Allows differential driving.
 * 
 * This library makes use of PWM module 0, outputs 0 and 3.
 * These are produced by PWM module 0's generators numbered 0 and 1.
 * 
 * Conventions:
 *  - Definition of "left" versus "right":
 *      - The "left motor" is the motor on your left when the robot is on the ground with the servo pointed away from you.
 *      - The "right motor" is the motor on your right when the robot is on the ground with the servo pointed away from you.
 *  - How to connect motors to motor board:
 *      - The left motor must be connected to motor port A.
 *      - The right motor must be connected to motor port B.
 *  - Motor pin assignments:
 *      - The red wire of the left motor will be connected to A- and is controlled by PB6
 *      - The black wire of the left motor will be connected to A+ and is controlled by PB7
 *      - The red wire of right motor will be connected to B- and is controlled by PB4
 *      - The black wire of the right motor will be connected to B+ and is controlled by PB5
 *  - Pin configurations:
 *      - A- (PB6) and B+ (PB5) will be configured as PWM outputs
 *      - A+ (PB7) and B- (PB4) will be configured as digital outputs.
 *      - We alternate + and - so that when both motors are driving forward (i.e. they are rotating OPPOSITE directions)
 *        with the same torque, their digital and PWM output configurations will be identical.
 *  - H-Bridge convention:
 *      - A value of 1 (high) on any of PB4/5/6/7 will connect the corresponding motor terminal (A/B/+/-) to battery power.
 *      - A value of 0 (low) on any of PB4/5/6/7 will connect the corresponding motor terminal (A/B/+/-) to ground.
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef _MOTORS_H_
#define _MOTORS_H_


#include <stdint.h>

/**
 * @brief Initialize the robot motors.
 * 
 */
void Motors_Init(void);


/**
 * @brief Set the torque for each of the two motors atomically
 * 
 * @param left_trq   Torque for left motor. Between -50 and 50.
 *                   Positive argument indicates forward motion of robot, negative indicates backward.
 *                   Zero indicates no rotation.
 * @param right_trq   Torque for right motor. Between -50 and 50.
 *                    Positive argument indicates forward motion of robot, negative indicates backward.
 *                    Zero indicates no rotation.
 */
void Motors_SetTorque(int16_t left_trq, int16_t right_trq);


/**
 * @brief Set the torque of the left motor individually
 * 
 * @param left_trq   Torque for left motor. Between -50 and 50.
 *                   Positive argument indicates forward motion of robot, negative indicates backward.
 *                   Zero indicates no rotation.
 */
void Motors_SetTorque_Left(int16_t left_trq);


/**
 * @brief Set the torque of the left motor individually
 * 
 * @param right_trq   Torque for right motor. Between -50 and 50.
 *                    Positive argument indicates forward motion of robot, negative indicates backward.
 *                    Zero indicates no rotation.
 */
void Motors_SetTorque_Right(int16_t right_trq);


/**
 * @brief Brake both motors (tie both motors' leads to ground)
 * 
 */
void Motors_Brake(void);


/**
 * @brief Brake left motor (tie motor leads to ground)
 * 
 */
void Motors_Brake_Left(void);


/**
 * @brief Brake right motor (tie motor leads to ground)
 * 
 */
void Motors_Brake_Right(void);


#endif // _MOTORS_H_


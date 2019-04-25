
#ifndef _MOTORCAN_H_
#define _MOTORCAN_H_

#include <stdint.h>
#include "can0.h"


void CAN_RightMotorTorch(int right);


void CAN_LeftMotorTorch(int left);


void CAN_MotorTorch(int left, int right);


void CAN_Servo(int value);


void CAN_Motorstart();
  

void CAN_Motorstop();

#endif // _MOTORCAN_H_

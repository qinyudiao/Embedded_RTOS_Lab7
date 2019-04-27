#include "DirectionCtrl.h"
#include "motorcan.h"



void TurnLeft90(void)
{
  CAN_MotorTorch(0,50);
  CAN_Servo(0);
 }
void TurnRight90(void)
{
  CAN_MotorTorch(50,0);
  CAN_Servo(180);
}
void TurnLeft60(void)
{
  CAN_MotorTorch(40,50);
  CAN_Servo(30);
}


void TurnRight60(void)
{
  CAN_MotorTorch(50,40);
  CAN_Servo(150);
}
void SlightLeft(int n)
{
  if(n>60)
    n=60;
  CAN_MotorTorch(40-n,40);
  CAN_Servo(90);
}
void SlightRight(int n)
{
  if(n>60)
    n=60;
  CAN_MotorTorch(40,40-n);
  CAN_Servo(90);
}

void Straight(void)
{
  CAN_Servo(90);
  CAN_MotorTorch(40,40);
}

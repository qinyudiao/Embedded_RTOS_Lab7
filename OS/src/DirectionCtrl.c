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
  
  if(n>400)
    n=400;
  if(n>150){
    int i = n/10;
    CAN_Servo(90+i);
  }
  else
    CAN_Servo(90);
  if(n>300)
    n=300;
  CAN_MotorTorch(250-n,250);
  
}
void SlightRight(int n)
{
  if(n>400)
    n=400;
  
  if(n>150){
    int i = n/10;
    CAN_Servo(80-i);
  }
  else
    CAN_Servo(85);
  if(n>300)
    n = 300;
  CAN_MotorTorch(250,250-n);

}

void Straight(void)
{
  CAN_Servo(90);
  CAN_MotorTorch(250,250);
}

void Back(void)
{
  CAN_Servo(90);
  CAN_MotorTorch(-250,-250);
}
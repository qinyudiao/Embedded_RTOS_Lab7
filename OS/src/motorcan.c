#include "motorcan.h"

void CAN_RightMotorTorch(int right)
{
  CAN0_SendDatawithIdx((uint8_t *)&right,0);
}


void CAN_LeftMotorTorch(int left){
  CAN0_SendDatawithIdx((uint8_t *)&left,1);
}

void CAN_MotorTorch(int left, int right){
  int data = (left<<16) + right;
  CAN0_SendDatawithIdx((uint8_t *)&data,3);
}


void CAN_Servo(int value){
  CAN0_SendDatawithIdx((uint8_t *)&value,2);
}

void CAN_Motorstop()
{
  int value = 2;
  CAN0_SendDatawithIdx((uint8_t *)&value,4);
}

void CAN_Motorstart()
{
  int value = 1;
  CAN0_SendDatawithIdx((uint8_t *)&value,4);
}
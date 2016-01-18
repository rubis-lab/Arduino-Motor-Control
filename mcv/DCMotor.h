#ifndef __DCMOTOR_H__
#define __DCMOTOR_H__
#include <Arduino.h>
#include <Servo.h>
//#include "RTSched.h"

#define MOTOR_PIN 7
#define DC_MOTOR_SENSITIVITY 5

int setMotorSpeed(int des_speed);
int setSpeed(int des_speed); 
void initDCMotor();


#endif
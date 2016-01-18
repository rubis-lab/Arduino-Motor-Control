#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__
#include <Arduino.h>
#include <Servo.h>
//#include "RTSched.h"

#define MOTOR_PIN 10
#define DC_MOTOR_SENSITIVITY 5

#define DC_MOTOR_DEBUG

int setMotorSpeed(int des_speed);
int setSpeed(int des_speed); 
void initDCMotor();


#endif
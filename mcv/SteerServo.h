#ifndef __STEERSERVO_H__
#define __STEERSERVO_H__
#include <Arduino.h>
#include <Servo.h> 
//#include "RTSched.h"

#define SERVO_PIN 9
#define STEER_SERVO_SENSITIVITY 5

void initSteerServo();
int setServoAngle(int des_angle);
int setAngle(int des_angle);

#endif

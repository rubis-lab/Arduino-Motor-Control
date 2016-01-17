#ifndef __DCMOTOR_H__
#define __DCMOTOR_H__
#include <Servo.h>
#include "RTSched.h"

#define MOTOR_PIN 7
#define DC_MOTOR_SENSITIVITY 5

int setSpeed(int des_speed);
void initDCMotor();


#endif
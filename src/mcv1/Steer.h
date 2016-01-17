#ifndef __STEER_H__
#define __STEER_H__ 
#include <Servo.h> 
#include "RTSched.h"

#define SERVO_PIN 9
extern int volatile curr_ang;

int setAng(int des_ang);
int setAng(int des_ang, int agility);
int adjustAngOverTime(int des_ang, int inc, int myid);

int steer_init();

#endif

#ifndef __RTSCHED_H__
#define __RTSCHED_H__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdlib.h>
#include "../lib/LinkedList.h"

// NOTE: timer2 is used in PWM 3, 11, and tone()
// using this library may trigger unexpected behaviors

//extern unsigned int volatile tim2_cnt;

//TODO: need to extend this to accomodate all typs
typedef int (*fnptr)(int, int, int);
typedef struct {
	fnptr fp;
	int var1;
	int var2;
} TaskStruct;

int RTSched_size();
int RTSched_clear();
int RTSched_add(TaskStruct);
int RTSched_remove(int);
int RTSched_dispatch();
int RTSched_init();
ISR(TIMER2_OVF_vect);

#endif
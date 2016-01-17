#ifndef __RTSCHED_H__
#define __RTSCHED_H__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdlib.h>
#include "../lib/LinkedList.h"

// NOTE: Timer2 is used in PWM 3, 11, and tone()
// using this library may trigger unexpected behaviors

//TODO: Need to extend this to accomodate all typs
//	Also need to support class functions as well
typedef int (*fnptr)(int, int, int);
typedef struct {
	fnptr fp;
	int var1;
	int var2;
} TaskStruct;

class RTSched 
{
private:
	LinkedList<TaskStruct> tim2_task_list;
public:
	int size();
	int clear();
	int add(TaskStruct ts);
	int remove(int idx);
	int dispatch();
	RTSched();
};

// NOTE: Static class instance
extern RTSched rts;

//ISR(TIMER2_OVF_vect);

#endif
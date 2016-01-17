#include "Steer.h"


int volatile curr_ang;
Servo sv;

int adjustAngOverTime(int des_ang, int inc, int myid) {
	if(curr_ang < des_ang) {
		sv.write(curr_ang + inc);
	}
	if(curr_ang > des_ang) {
		sv.write(curr_ang - inc);
	}
	if(curr_ang == des_ang) {
		RTSched_remove(myid);
		return curr_ang;
	} 
	return curr_ang;
}

int setAng(int des_ang) {
  //sv.write(des_ang);
  curr_ang = des_ang;
  return curr_ang;
}

int setAng(int des_ang, int agility) {
	int inc = 5;
	fnptr myfp = adjustAngOverTime;
	TaskStruct sp;
	sp.fp = myfp;
	sp.var1 = des_ang;
	sp.var2 = agility;
	RTSched_add(sp);
	return curr_ang;
}

int steer_init() {
  sv.attach(SERVO_PIN); 
}
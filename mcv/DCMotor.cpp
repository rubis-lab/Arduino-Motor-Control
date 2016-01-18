#include "DCMotor.h"

Servo dc_mt;
int volatile curr_speed;

int adjustSpeedOverTime(int des_speed, int inc, int myid) {
	if(curr_speed < des_speed) {
		curr_speed += inc;
		dc_mt.write(curr_speed);
	}
	if(curr_speed > des_speed) {
		curr_speed -= inc;
		dc_mt.write(curr_speed);
	}
	if(curr_speed == des_speed) {
		//rts.remove(myid);
		return curr_speed;
	} 
	return curr_speed;
}

int setMotorSpeed(int des_speed) {
	/*
	fnptr myfp = adjustSpeedOverTime;
	TaskStruct sp;
	sp.fp = myfp;
	sp.var1 = des_speed;
	sp.var2 = DC_MOTOR_SENSITIVITY;
	//rts.add(sp);
	*/
	return 1;
}
int setSpeed(int des_speed) {
	dc_mt.write(des_speed);
	curr_speed = des_speed;
	return curr_speed;
}
void initDCMotor() {
	dc_mt.attach(MOTOR_PIN);
	return;
}
#include "SteerServo.h"

Servo st_sv;
int volatile curr_angle;

int adjustAngleOverTime(int des_angle, int inc, int myid) {
	if(curr_angle < des_angle) {
		st_sv.write(curr_angle + inc);
	}
	if(curr_angle > des_angle) {
		st_sv.write(curr_angle - inc);
	}
	if(curr_angle == des_angle) {
		rts.remove(myid);
		return curr_angle;
	} 
	return curr_angle;
}

int setServoAngle(int des_angle) {
	fnptr myfp = adjustAngleOverTime;
	TaskStruct sp;
	sp.fp = myfp;
	sp.var1 = des_angle;
	sp.var2 = STEER_SERVO_SENSITIVITY;
	rts.add(sp);

	return 1;
}

void initSteerServo() {
	st_sv.attach(SERVO_PIN);
	return; 
}



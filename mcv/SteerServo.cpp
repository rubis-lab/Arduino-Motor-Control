#include "SteerServo.h"

Servo st_sv;
int volatile curr_angle;

int adjustAngleOverTime(int des_angle, int inc, int myid) {
	if(curr_angle < des_angle) {
		curr_angle += inc;
		st_sv.write(curr_angle);
	}
	if(curr_angle > des_angle) {
		curr_angle -= inc;
		st_sv.write(curr_angle);
	}
	if(curr_angle == des_angle) {
		//rts.remove(myid);
		return curr_angle;
	} 
	//rts.enqueue();
	return curr_angle;
}

int setServoAngle(int des_angle) {
	/*
	fnptr myfp = adjustAngleOverTime;
	TaskStruct sp;
	sp.fp = myfp;
	sp.var1 = des_angle;
	sp.var2 = STEER_SERVO_SENSITIVITY;
	//rts.add(sp);
	*/
	return 1;
}

int setAngle(int des_angle) {
	st_sv.write(des_angle);
	curr_angle = des_angle;
#ifdef STEER_SERVO_DEBUG
	Serial.print("Curr angle : ");
	Serial.println(curr_angle);
#endif
	return curr_angle;
}


void initSteerServo() {
	st_sv.attach(SERVO_PIN);
	st_sv.write(STEER_SERVO_CENTER);
#ifdef STEER_SERVO_DEBUG
	Serial.begin(19200);
	Serial.println("Steer servo debug mode : ");
#endif
	return; 
}



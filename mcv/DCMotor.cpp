#include "DCMotor.h"

//Servo dc_mt;
int volatile curr_speed;
/*
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
*/
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
	analogWrite(MOTOR_PIN, des_speed);
	curr_speed = des_speed;
#ifdef DC_MOTOR_DEBUG
	Serial.print("Curr speed : ");
	Serial.println(curr_speed);
#endif
	return curr_speed;
}
void initDCMotor() {
	//dc_mt.attach(MOTOR_PIN);
	pinMode(MOTOR_PIN, OUTPUT);
	analogWrite(MOTOR_PIN, 0);
#ifdef DC_MOTOR_DEBUG
	Serial.begin(19200);
	Serial.println("DC motor debug mode : ");
#endif
	return;
}
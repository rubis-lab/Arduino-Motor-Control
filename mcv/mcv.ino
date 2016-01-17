/*
 * Motor Control Version 1
 * 2016-01-16
 */
#include "src/ArduinoI2C.h"
#include "src/RTSched.h"
#include "src/SteerServo.h"
#include "src/DCMotor.h"


#define DEBUG_
#undef DEBUG_

void setup() {
  ArduinoI2C();
  RTSched();
#ifdef DEBUG_
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("DEBUG mode");
#endif
  initSteerServo();
  initDCMotor();
}

void loop() {

#ifdef DEBUG_
  int cmd;
  Serial.println("Servo angle (0-180): ");
  while(!Serial.available());
  cmd = Serial.parseInt();
  Serial.print("Applied: ");
  Serial.println(cmd);
#else

#endif


}

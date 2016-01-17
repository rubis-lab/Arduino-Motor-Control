/*
 * Motor Control Version 1
 * 2016-01-16
 */

#include "Steer.h"
#include "RTSched.h"

#define MOTOR_PIN 11
#define DEBUG_

void setup() {
  RTSched_init();
#ifdef DEBUG_
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("DEBUG mode");
#endif
  steer_init();
}

void loop() {

#ifdef DEBUG_
  int cmd;
  Serial.println("Servo angle (0-180): ");
  while(!Serial.available());
  cmd = Serial.parseInt();
  //shiftangle(cmd);
  //sv.write(cmd);
  Serial.print("Applied: ");
  Serial.println(cmd);
#else

#endif


}

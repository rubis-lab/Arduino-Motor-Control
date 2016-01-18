/*
 * Motor Control Version 1
 * 2016-01-16
 */
 
#include "ArduinoI2C.h"
#include "SteerServo.h"
#include "DCMotor.h"

#define DEBUG_SERIAL
#undef DEBUG_SERIAL
#define DEBUG_LED

void setup() {
  initArduinoI2C();
  initSteerServo();
  initDCMotor();
#ifdef DEBUG_SERIAL
  Serial.begin(19200);
  Serial.setTimeout(50);
  Serial.println("Serial debug mode");
#endif
#ifdef DEBUG_LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
#endif
}

void loop() {
  if(i2c_status == 0) {
    if(i2c_type == 1) {
#ifdef DEBUG_LED
      digitalWrite(13, HIGH);
#endif
      setAngle(i2c_data);
    }
    if(i2c_type == 2) {
#ifdef DEBUG_LED
      digitalWrite(13, LOW);
#endif
      setSpeed(i2c_data);
    }
  }
#ifdef DEBUG_SERIAL
  //Serial.println("loop cnt");
#endif
  delay(1000);
}

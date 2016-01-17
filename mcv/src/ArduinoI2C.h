#ifndef __ARDUINOI2C_H__
#define __ARDUINOI2C_H__
#include <Wire.h>
#define SLAVE_ADDRESS 0x04

class ArduinoI2C {
private:
public:
	ArduinoI2C();
	static void rx(int byteCount);
	static void tx();

};
extern ArduinoI2C ai2c;

#endif
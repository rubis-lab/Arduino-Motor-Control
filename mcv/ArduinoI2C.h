#ifndef __ARDUINOI2C_H__
#define __ARDUINOI2C_H__
#include <Arduino.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04

extern int i2c_type;
extern int i2c_data;
void initArduinoI2C();
void i2c_rx(int byteCount);
void i2c_tx();

#endif
#include "ArduinoI2C.h"

int i2c_type;
int i2c_data;

void initArduinoI2C()
{
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(i2c_rx);
	Wire.onRequest(i2c_tx);
	//Serial.begin(19200);
}

void i2c_rx(int byteCount) {
	if(Wire.available()) {
		i2c_type = Wire.read();
		//i2c_data = Wire.read();
		//i2c_data += Wire.read() << 8;
		/*
		Serial.print("received : ");
		Serial.println(i2c_type);
		*/
	}
	i2c_tx();
 	return;
}

void i2c_tx() {
	Wire.write(i2c_type);
}

#include "ArduinoI2C.h"

int i2c_type;
int i2c_data;
int i2c_status;

void initArduinoI2C()
{
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(i2c_rx);
	Wire.onRequest(i2c_tx);
	i2c_status = 0;
#ifdef I2C_DEBUG
	Serial.begin(19200);
	Serial.println("i2c debug");
#endif

}

void i2c_rx(int byteCount) {
	while(Wire.available()) {
		switch(i2c_status) {
		case 0:
			i2c_type = Wire.read();
			i2c_status++;
			break;
		case 1: 
			i2c_data = Wire.read();
			i2c_status++;
			break;
		case 2: 
			i2c_data += Wire.read() << 8;
			i2c_status = 0;
#ifdef I2C_DEBUG
			Serial.print("type : ");
			Serial.println(i2c_type);
			Serial.print("data : ");
			Serial.println(i2c_data);
#endif
			i2c_tx();
			break;
		default:
			break;
		}
	}
 	return;
}

void i2c_tx() {
	Wire.write(i2c_type);
}

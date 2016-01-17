#include "ArduinoI2C.h"
ArduinoI2C ai2c;

ArduinoI2C::ArduinoI2C()
{
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(ArduinoI2C::rx);
	Wire.onRequest(ArduinoI2C::tx);
	
}

void ArduinoI2C::rx(int byteCount) {
	int number;
 	while(Wire.available()) {
  	number = Wire.read();
	  if (number == 1){
	   if (state == 0){
	    digitalWrite(13, HIGH); // set the LED on
	    state = 1;
	   } else{
	    digitalWrite(13, LOW); // set the LED off
	    state = 0;
	   }
	  }
	  if(number==2) {
	   number = (int)temp;
	  }
 	}
 	return;
}

void ArduinoI2C::tx() {
	Wire.write(0);
}
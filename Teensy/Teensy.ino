/* 
 * This is the primary controller code (Teensy 3.6) for the 2019 OSU USLI team rover
 *
 * Build using Teensyduino (https://www.pjrc.com/teensy/td_download.html) through Arduino IDE
 * or a Makefile script could be made implementing https://github.com/PaulStoffregen/teensy_loader_cli/
*/

#include <Arduino.h>

#define HWSERIAL Serial1 // Define hardware serial port to use for Teensy 3.6 (Serial 1 - 6)

void setup() {
	// Set up baud rate for serial communication
	Serial.begin(9600);
	HWSERIAL.begin(9600);
}

void loop(){
	int incomingByte;
	constexpr unsigned MESSAGE_LEGNTH = 1;
	constexpr uint8_t message[MESSAGE_LEGNTH] = {15}; // Could also be a single byte or a string

	if(Serial.available() > 0){ // Check number of received bytes in buffer
		incomingByte = Serial.read(); // Reads first byte received
		Serial.print("USB received: ");
		Serial.println(incomingByte, DEC); // Format with a base or number of decimal places (for floating-point numbers)
	}

	if(HWSERIAL.available() > 0){
		incomingByte = HWSERIAL.read();
		Serial.print("UART received: ");
		Serial.println(incomingByte, DEC);
	}

	Serial1.write(message, MESSAGE_LEGNTH);
}

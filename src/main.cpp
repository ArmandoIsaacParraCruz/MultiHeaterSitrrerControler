#include <Arduino.h>
#include <MCP3XXX.h>
#include "max6675.h"


const int ssPin = 5; 
int myArray[6] = {0, 0, 0, 0, 0, 0}; 

void setup() {
	Serial.begin(9600);
	pinMode(ssPin, OUTPUT);
	SPI.begin(6, 7, 8, 5);
}

void loop() {
	for (uint8_t i = 0; i < 6; i++) {

		for (uint8_t j = 0; j < 6; j++) {
			if(j == i) {
				myArray[j] = 1;
			} else {
				myArray[j] = 0;
			}
			
			Serial.print(myArray[j]);Serial.print("	");
			int dataReceived = SPI.transfer(myArray[j]);
			Serial.println("Dato enviado: " + String(myArray[j]) + ", Dato recibido: " + String(dataReceived));
			delay(25);
			
		}
		delay(1000);
		Serial.println();
		//int dataToSend = myArray[i];
		//int dataReceived = SPI.transfer(dataToSend);
		//Serial.println("Dato enviado: " + String(dataToSend) + ", Dato recibido: " + String(dataReceived));
		
	}
}



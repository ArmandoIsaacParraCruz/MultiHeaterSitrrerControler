#include <Arduino.h>
#include "StirringController.h"

// Definir los pines para el motor y el encoder
const uint8_t motorPin = 20;  // Ejemplo: Pin GPIO12
const uint8_t encoderPin = 1; // Ejemplo: Pin GPIO14
uint32_t currentTime;
uint32_t counter;

// Declarar una función de interrupción (puedes definirla según tus necesidades)
void IRAM_ATTR interruptFunction();

// Definir los parámetros del controlador
const uint8_t channel = 0;
const float pulsesPerRevolution = 70; // Ejemplo: 70 pulsos por revolución
const uint32_t frequency = 1000; // Ejemplo: Frecuencia de 1 kHz
const uint8_t resolution = 8; // Ejemplo: Resolución de 8 bits
const double kp = 1.0;
const double ki = 0.1;
const double kd = 0.1;

// Crear una instancia de StirringController
StirringController stirringController(kp, ki, kd);

void setup() {
  Serial.begin(115200);
  currentTime = millis();
  stirringController.begin(motorPin, encoderPin, channel, pulsesPerRevolution, frequency, resolution, interruptFunction);
  stirringController.setOutputLimits(0.0, 255.0);
  stirringController.setSetpoint(255);
  counter = 0;
}

void loop() {
  if(millis() - currentTime >= 200) {
    stirringController.updateInput();
    stirringController.adjustOutputSignal();
    Serial.println();
    Serial.println(stirringController.getInput());
    Serial.println(stirringController.getOutput());
    Serial.println((counter*5.0*60)/70.0);
    counter = 0;
    currentTime = millis();
  }
}


void IRAM_ATTR interruptFunction()
{
  stirringController.incrementPulses();
  ++counter; 
}

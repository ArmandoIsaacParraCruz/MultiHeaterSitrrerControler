/*#include <Arduino.h>
#include "HeatingController.h"
#include "MCP23017.h"

#define debounce 8
#define MCP23017_ADDRESS 0x20 // Dirección del MCP23017 (A0-A2 conectados a GND)
#define SDA 8
#define SCL 18
#define CS 0

const double kp = 1.0;
const double ki = 0.1;
const double kd = 0.1;
const uint8_t heatingResistorPin = 4;

void IRAM_ATTR zeroCrossingDetection();
// Definir el pin para la interrupción externa
const int interruptPin = 14;

// Contador de interrupciones
volatile int interruptCount = 0;

uint32_t debounceTime;

uint32_t currentTime;

HeatingController heatingController(kp, ki, kd);

bool blink = true;


void setup() {
  // Iniciar comunicación serial
  Serial.begin(115200);
  SPI.begin();
  // Configurar el pin para la interrupción externa
  pinMode(interruptPin, INPUT_PULLUP);
  
  // Adjuntar la función de interrupción al pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), zeroCrossingDetection, FALLING);

  heatingController.begin(heatingResistorPin, CS);
  heatingController.setOutputLimits(0, 120);
  heatingController.setSetpoint(77);

  MCP23017::begin(SDA, SCL, MCP23017_ADDRESS);
  

  MCP23017::digitalWrite(0, HIGH);
  MCP23017::digitalWrite(1, HIGH);
  MCP23017::digitalWrite(2, HIGH);
  MCP23017::digitalWrite(3, HIGH);
  MCP23017::digitalWrite(4, HIGH);
  MCP23017::digitalWrite(5, HIGH);
  MCP23017::digitalWrite(8, HIGH);
  MCP23017::digitalWrite(9, HIGH);

  MCP23017::digitalWrite(12, HIGH);
  
  currentTime = millis();
  debounceTime = millis();
}

void loop() {
  if(millis() - currentTime >= 1000) {
    if(blink) {
      MCP23017::digitalWrite(12, HIGH);
    } else {
      MCP23017::digitalWrite(12, LOW);
    }
    blink = !blink;
   
    heatingController.updateInput();
    
    heatingController.adjustOutputSignal();
    Serial.println(heatingController.getInput());
    Serial.println(heatingController.getOutput());
    Serial.println();
    currentTime = millis();
  }
}

// Función de manejo de la interrupción
void IRAM_ATTR zeroCrossingDetection() {
  if(millis() - debounceTime >= debounce) {
    if(heatingController.getSemicyclesCounter() > 120) {
      heatingController.setSemicyclesCounter(0);
    } else {
      heatingController.incrementSemicyclesCounter();
    }
    heatingController.adjustOutputSignal();
    debounceTime = millis();
  }
}



#include <Arduino.h>
#include "HeatingController.h"
#include "MCP23017.h"

#define debounce 8
#define MCP23017_ADDRESS 0x20 // Dirección del MCP23017 (A0-A2 conectados a GND)
#define SDA 8
#define SCL 18
#define CS1 0
#define CS2 1
#define LED 12

const double kp = 1.0;
const double ki = 0.1;
const double kd = 0.1;
const uint8_t heatingResistorPin = 4;
const uint8_t pin = 10;

void IRAM_ATTR zeroCrossingDetection();
// Definir el pin para la interrupción externa
void toggle();

const int interruptPin = 14;

// Contador de interrupciones
volatile int interruptCount = 0;

uint32_t debounceTime;

uint32_t currentTime;

HeatingController heatingController(kp, ki, kd);
HeatingController measurer(kp, ki, kd);

bool blink = true;


void setup() {
  // Iniciar comunicación serial
  Serial.begin(115200);
  SPI.begin();
  // Configurar el pin para la interrupción externa
  pinMode(interruptPin, INPUT_PULLUP);
  
  // Adjuntar la función de interrupción al pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), zeroCrossingDetection, FALLING);

  heatingController.begin(heatingResistorPin, CS1);
  heatingController.setOutputLimits(0, 120);
  heatingController.setSetpoint(77);

  measurer.begin(pin, CS2);
  measurer.setOutputLimits(0, 120);
  measurer.setSetpoint(77);

  MCP23017::begin(SDA, SCL, MCP23017_ADDRESS);
  

  MCP23017::digitalWrite(0, HIGH);
  MCP23017::digitalWrite(1, HIGH);
  MCP23017::digitalWrite(2, HIGH);
  MCP23017::digitalWrite(3, HIGH);
  MCP23017::digitalWrite(4, HIGH);
  MCP23017::digitalWrite(5, HIGH);
  MCP23017::digitalWrite(8, HIGH);
  MCP23017::digitalWrite(9, HIGH);

  MCP23017::digitalWrite(LED, HIGH);
  
  currentTime = millis();
  debounceTime = millis();
  while (true)
  {
    if(Serial.available() > 0){
      String inputString = Serial.readString();
      if(inputString == "A") {
        break;
      }
    }
  }
}

void loop() {
  if(millis() - currentTime >= 1000) {

    toggle();
   
    heatingController.updateInput();
    measurer.updateInput();
    
    heatingController.adjustOutputSignalManually(30);
    Serial.print(heatingController.getInput());
    Serial.print("  ");
    Serial.println(measurer.getInput());
    currentTime = millis();
  }
}

// Función de manejo de la interrupción
void IRAM_ATTR zeroCrossingDetection() {
  if(millis() - debounceTime >= debounce) {
    if(heatingController.getSemicyclesCounter() > 120) {
      heatingController.setSemicyclesCounter(0);
    } else {
      heatingController.incrementSemicyclesCounter();
    }
    heatingController.adjustOutputSignal();
    debounceTime = millis();
  }
}


void toggle()
{
  if(blink) {
    MCP23017::digitalWrite(LED, HIGH);
  } else {
    MCP23017::digitalWrite(LED, LOW);
  }
  blink = !blink;
}*/
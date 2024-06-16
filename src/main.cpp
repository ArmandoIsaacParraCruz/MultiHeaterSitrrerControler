#include <Arduino.h>
#include "MultiHeaterStirrerController.h"

MultiHeaterStirrerController multiHeaterStirrerController{};
void interruptFunction1();
void interruptFunction2();
void interruptFunction3();
void interruptFunction4();
void interruptFunction5();
void interruptFunction6();
void zeroCrossingDetection();
void (*interruptFunctions[])(void)= { interruptFunction1, 
                                      interruptFunction2,
                                      interruptFunction3,
                                      interruptFunction4,
                                      interruptFunction5,
                                      interruptFunction6,
                                      zeroCrossingDetection};



void setup()
{
	Serial.begin(115200);
	SPI.begin();
	multiHeaterStirrerController.setupMultiHeaterStirrerController(interruptFunctions);
}

void loop()
{
    multiHeaterStirrerController.mainLoop();
}


void IRAM_ATTR interruptFunction1()
{
  
   multiHeaterStirrerController.stirringControllers.at(0).incrementPulses(); 
}

void IRAM_ATTR interruptFunction2()
{
	 multiHeaterStirrerController.stirringControllers.at(1).incrementPulses(); 
}

void IRAM_ATTR interruptFunction3()
{
	 multiHeaterStirrerController.stirringControllers.at(2).incrementPulses(); 
}

void IRAM_ATTR interruptFunction4()
{
	 multiHeaterStirrerController.stirringControllers.at(3).incrementPulses(); 
}

void IRAM_ATTR interruptFunction5()
{
	 multiHeaterStirrerController.stirringControllers.at(4).incrementPulses(); 
}

void IRAM_ATTR interruptFunction6()
{
	multiHeaterStirrerController.stirringControllers.at(5).incrementPulses(); 
}

void IRAM_ATTR zeroCrossingDetection() {
  if(millis() - MultiHeaterStirrerController::zeroCrossingDebounceTimer >= ZERO_CROSSING_DEBOUNCE_TIME) {
    
    if(HeatingController::getSemicyclesCounter() > multiHeaterStirrerController.MAX_LIMIT_SEMICYCLE_VALUE) {
      HeatingController::setSemicyclesCounter(multiHeaterStirrerController.MIN_LIMIT_SEMICYCLE_VALUE);
    } else {
      HeatingController::incrementSemicyclesCounter();
    }

    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
      multiHeaterStirrerController.heatingControllers.at(i).adjustOutputSignal();
    }

    MultiHeaterStirrerController::zeroCrossingDebounceTimer = millis();
  }
}


/*
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
}

void loop() {
  // Chill
}





*/

/*
#include <MCP3XXX.h>
#include <SPI.h>


MCP3008 adc1;
MCP3008 adc2;

const int SS_MAX = 6;
const int RESET_ATMEGA = 9;
const uint8_t SS_ATMEGA = 10;
const int pinInterrupcion1 = 1;
const int pinInterrupcion2 = 2;
const int pinInterrupcion3 = 42;
const int pinInterrupcion4 = 41;
const int pinInterrupcion5 = 40;
const int pinInterrupcion6 = 39;
uint32_t cuenta1;
uint32_t cuenta2;
uint32_t cuenta3;
uint32_t cuenta4;
uint32_t cuenta5;
uint32_t cuenta6;
uint32_t lastTime;


void atmegaCommunication(){
  if (Serial.available()) {
    char comando = Serial.read();
    digitalWrite(SS_ATMEGA, LOW);
    if (comando == '1') {
      SPI.transfer('1');
      Serial.println("Comando enviado: Encender LED en el esclavo");
    } else if (comando == '0') {
      digitalWrite(SS_ATMEGA, LOW);
      SPI.transfer('0');
      Serial.println("Comando enviado: Apagar LED en el esclavo");
    }
  }
  digitalWrite(SS_ATMEGA, HIGH);
}


float readMAX6675() {

  digitalWrite(SS_MAX, LOW); // Activar sensor
  uint16_t v;

  // Leer datos
  byte msb = SPI.transfer(0x00);
  byte lsb = SPI.transfer(0x00);

  digitalWrite(SS_MAX, HIGH); // Desactivar sensor

  // Convertir bits a temperatura
  int temperature = ((msb << 8) | lsb) >> 3;

  float celsius = temperature * 0.25;
  return celsius;
}

void miRutinaInterrupcion1() {
  ++cuenta1;
}
void miRutinaInterrupcion2() {
  ++cuenta2;
}
void miRutinaInterrupcion3() {
  ++cuenta3;
}
void miRutinaInterrupcion4() {
  ++cuenta4;
}
void miRutinaInterrupcion5() {
  ++cuenta5;
}
void miRutinaInterrupcion6() {
  ++cuenta6;
}


void setup()
{
  Serial.begin(115200);
  pinMode(SS_MAX, OUTPUT);
  pinMode(SS_ATMEGA, OUTPUT);
  pinMode( RESET_ATMEGA, OUTPUT);
  digitalWrite(SS_ATMEGA, HIGH);
  digitalWrite(RESET_ATMEGA, HIGH);
  digitalWrite(SS_MAX, HIGH); // Establecer SS en alto para que el esclavo esté inactivo inicialmente
  SPI.begin();

  adc1.begin(4);
  adc2.begin(5);

  pinMode(pinInterrupcion1, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion1), miRutinaInterrupcion1, FALLING);
  pinMode(pinInterrupcion2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion2), miRutinaInterrupcion2, FALLING);
  pinMode(pinInterrupcion3, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion3), miRutinaInterrupcion3, FALLING);
  pinMode(pinInterrupcion4, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion4), miRutinaInterrupcion4, FALLING);
  pinMode(pinInterrupcion5, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion5), miRutinaInterrupcion5, FALLING);
  pinMode(pinInterrupcion6, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion6), miRutinaInterrupcion6, FALLING);

    cuenta1 = 0;
    cuenta2 = 0;
    cuenta3 = 0;
    cuenta4 = 0;
    cuenta5 = 0;
    cuenta6 = 0;
  lastTime = millis();
}

void loop()
{
  if(millis() - lastTime >= 1000) {
    Serial.print("ADC1:");
    Serial.print(adc1.analogRead(0));
    Serial.print("  Temperatura Sensor Infrarrojo:");

    long Vin = map(adc2.analogRead(4), 0, 1023, 0, 3300);
    float infraredTemp = (((600.0)/(3.3-0.660)) * (Vin - 660))/1000.0 + 3;
    Serial.print(infraredTemp);
    Serial.print("  Temperatura Termopar:");
    Serial.println(readMAX6675());
    atmegaCommunication();

    cuenta1 = 0;
    cuenta2 = 0;
    cuenta3 = 0;
    cuenta4 = 0;
    cuenta5 = 0;
    cuenta6 = 0;
    lastTime = millis();
    //SPI.transfer('1');
    //  Serial.println("Comando enviado: Encender LED en el esclavo");
  }

}

*/

/*FUNCIONA !!!!!!!!
#include <ESP32SPISlave.h>
#include <SPI.h>

const uint8_t CS = 42;
const uint8_t MISO_ESP32 = 41;
const uint8_t MOSI_ESP32 = 40;
const uint8_t CLK = 39;


constexpr uint32_t BUFFER_SIZE {1};

uint8_t spi_rx_buf[BUFFER_SIZE];
uint8_t spi_tx_buf[BUFFER_SIZE];



ESP32SPISlave slave;

void setup() {

    Serial.begin(115200);
    slave.begin(HSPI, CLK, MISO_ESP32, MOSI_ESP32, CS);
    memset(spi_rx_buf, 0, BUFFER_SIZE);
  memset(spi_tx_buf, 0, BUFFER_SIZE);
}

void loop() {

  slave.wait(spi_rx_buf, spi_tx_buf, BUFFER_SIZE);

  if (slave.available()) {

    Serial.print("Dato recibido: ");
    Serial.println(spi_rx_buf[0]);
    slave.pop();

  }

}
*/


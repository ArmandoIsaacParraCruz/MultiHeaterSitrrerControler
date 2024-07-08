#include "MultiHeaterStirrerController.h"
uint32_t MultiHeaterStirrerController::zeroCrossingDebounceTimer;

MultiHeaterStirrerController::MultiHeaterStirrerController()
{
    
}


void MultiHeaterStirrerController::setupMultiHeaterStirrerController(void (*interruptFunctions[])(void))
{
    Serial.begin(115200);
    MCP23017::begin(SDA, SCL, MCP23017_ADDRESS);

    for (int i = 0; i < NUMBER_OF_PLACES; ++i) {
        MCP23017::digitalWrite(CS_MAX6675[i], HIGH);
    }

    MCP23017::digitalWrite(STIRRING_KNOBS, HIGH);
    MCP23017::digitalWrite(HEATING_KNOBS, HIGH);
    MCP23017::digitalWrite(LED, HIGH);

    SPI.begin();

    automaticProcessStatus = AutomaticProcessStatus::PendingDataSubmission;
    gpio_install_isr_service(0);


    for (int i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.emplace_back(StirringController(stirringKp[i],
                                                            stirringKi[i],
                                                            stirringKd[i]));
    }

    for (int i = 0; i < NUMBER_OF_PLACES; ++i) {
        heatingControllers.emplace_back(HeatingController(  heatingKp[i], 
                                                            heatingKi[i], 
                                                            heatingKd[i],
                                                            alpha[i],
                                                            beta[i],
                                                            tau1[i],
                                                            tau2[i]));
    }
    

    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).begin(MOTOR_A_PINS[i], 
                                        ENCODER_PHASE_A_PINS[i], 
                                        i, 
                                        PULSES_PER_REVOLUTION,
                                        FRECUENCY,
                                        PWM_RESOLUTION,
                                        interruptFunctions[i]);
        stirringControllers.at(i).setOutputLimits(double(MIN_LIMIT_PWM_VALUE), double(MAX_LIMIT_PWM_VALUE));
        stirringControllers.at(i).setSetpoint(0);

        heatingControllers.at(i).begin(RESISTORS_PINS[i], CS_MAX6675[i]);
        heatingControllers.at(i).setOutputLimits(MIN_LIMIT_SEMICYCLE_VALUE, MAX_LIMIT_SEMICYCLE_VALUE);
        heatingControllers.at(i).setSetpoint(0);
    }


    
    
    pinMode(ZERO_CROSSING_PIN, INPUT_PULLUP);
    attachInterrupt(ZERO_CROSSING_PIN, interruptFunctions[6], RISING);

    pinMode(operationModeButtonPin, INPUT_PULLUP);
    operationMode = readOperationModeButton();
    RemoteCommunication_2::beginRemoteCommunication();
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);

    lastManualStirringAdjustmentTime = millis();
    lastManualHeatingAdjustmentTime = millis();
    lastSendHMIManualAdjustmentMeasurementsTime = millis();
    zeroCrossingDebounceTimer = millis();
    Serial.println("READY_SETUP");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // Imprimir la dirección MAC en el formato requerido
    Serial.print("uint8_t RemoteCommunication_2::mac_HMI[6] = {");
    for (int i = 0; i < 6; i++) {
        Serial.print("0x");
        Serial.print(mac[i], HEX);
        if (i < 5) {
        Serial.print(", ");
        }
    }
    Serial.println("};");
}

void MultiHeaterStirrerController::resetSettings()
{
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).adjustOutputSignalManually(0);
    }
    automaticProcessStatus = AutomaticProcessStatus::PendingDataSubmission;
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);
}

void MultiHeaterStirrerController::mainLoop()
{
    if(operationMode != readOperationModeButton()) {
        ESP.restart();
    } else if(operationMode == OperationMode::Automatic) {
        automaticProcess();
    } else if(operationMode == OperationMode::Manual) {
        manualProcess();
    }
}

void MultiHeaterStirrerController::automaticProcess()
{
    switch (automaticProcessStatus)
    {
    case AutomaticProcessStatus::PendingDataSubmission:
        PendingDataSubmission();
        break;
    
    case AutomaticProcessStatus::ProcessingDataReceived:
        ProcessingDataReceived();
        break;
    case AutomaticProcessStatus::AutomaticProcessInProgress:
        AutomaticProcessInProgress();
        break;
    default:
        break;
    }
}



void MultiHeaterStirrerController::PendingDataSubmission()
{
    if(RemoteCommunication_2::getProcessesSpecificationsMessageStatus() == ProcessMessageStatus::Received) {
        automaticProcessStatus = AutomaticProcessStatus::ProcessingDataReceived;
    }
}

void MultiHeaterStirrerController::ProcessingDataReceived()
{
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.println("temperatureSetpoints: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print("Inicial: ");
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[i].initialTemperature);
        Serial.print("  Final: ");
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[i].finalTemperature);
        Serial.print("  Funcion: ");
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[i].tempFunction);
        Serial.println("");
    }
    Serial.println("");

    Serial.println("stirringSetpoints: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[i]);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.print("processDuration: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.processDuration[i]);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.println(RemoteCommunication_2::processesSpecificationsMessage.configuredProcesses);

    currentProcess = 0;
    updatingSetponts();

    lastAutomaticStirringAdjustmentTime = millis();
    lastAutomaticHeatingAdjustmentTime = millis();
    lastAutomaticProcessTime = millis();
    lastSendHMIAutomaticAdjustmentMeasurementsTime = millis();

    automaticProcessStatus = AutomaticProcessStatus::AutomaticProcessInProgress;
}

void MultiHeaterStirrerController::updatingSetponts()
{
    Serial.println("updating setpoints");
    bool once = true;
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        if(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]) {
            Serial.print(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[currentProcess]);
            Serial.print(" ");
            stirringControllers.at(i).setSetpoint(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[currentProcess]);
            
            if(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].tempFunction == ramp && once) {
                generateRampSetponts();
                once = false;
            } else {
                heatingControllers.at(i).setSetpoint(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].initialTemperature);
            }
           
        }
    }
    Serial.println();
}

void MultiHeaterStirrerController::AutomaticProcessInProgress()
{
    if(millis() - lastAutomaticProcessTime >= RemoteCommunication_2::processesSpecificationsMessage.processDuration[currentProcess] * 1000 * 60) {
        evaluateProcessDuration();
        lastAutomaticProcessTime = millis();
    }
    
    if(millis() - lastAutomaticStirringAdjustmentTime >= TWO_HUNDRED_MILLISECONDS) {
        produceStirringPIDOutputs();
        lastAutomaticStirringAdjustmentTime = millis();
    }

    if(millis() - lastAutomaticHeatingAdjustmentTime  >= ONE_SECOND) {
        produceHeatingPIDOutputs();
        lastAutomaticHeatingAdjustmentTime  = millis();
    }

    if(millis() - lastSendHMIAutomaticAdjustmentMeasurementsTime  >= ONE_SECOND) {
        sendHMIAutomaticProcessesMeasurements();
        lastSendHMIAutomaticAdjustmentMeasurementsTime  = millis();
    }
    
}

void MultiHeaterStirrerController::evaluateProcessDuration()
{
        ++currentProcess;
        if(currentProcess <= RemoteCommunication_2::processesSpecificationsMessage.configuredProcesses) {
            Serial.println("next configured Process");
            updatingSetponts();
           
        } else {
            Serial.println("All the processes have finished");
            ESP.restart();
        }
}



void MultiHeaterStirrerController::produceStirringPIDOutputs()
{
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        if(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]) {
            stirringControllers.at(i).updateInput();
            stirringControllers.at(i).adjustOutputSignal();
        }
    }
}

void MultiHeaterStirrerController::produceHeatingPIDOutputs()
{
    float rampSetpoint;
    if(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].tempFunction == ramp) {
        if(!rampSetpoints.empty()) {
            rampSetpoint = rampSetpoints.front();
            rampSetpoints.erase(rampSetpoints.begin());
        }
    }

    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {

        if(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]) {
            if(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].tempFunction == ramp) {
                heatingControllers.at(i).setSetpoint(rampSetpoint);
            }
           heatingControllers.at(i).updateInput();
        }
        Serial.print(heatingControllers.at(i).getInput());
        Serial.print(" "); 
    }
    Serial.println();
}


void MultiHeaterStirrerController::sendHMIAutomaticProcessesMeasurements()
{
    uint16_t Vin= readADC(INFRAREF_HEATING_CHANNEL_SENSOR, ADC_INFRARRED_SENSOR);
    float temp =((5.0/22.0)*(Vin-660));
    measurements.infraredSensorTemp = temp;
    
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        measurements.RPM[i] = stirringControllers.at(i).getInput();
        measurements.temperatures[i] = heatingControllers.at(i).getInput();
    }
    measurements.timeInSencods = (millis() - lastAutomaticProcessTime) / 1000;
    RemoteCommunication_2::sendMeasurements(measurements);
}


void MultiHeaterStirrerController::manualProcess()
{
    if(millis() - lastManualStirringAdjustmentTime >= TWO_HUNDRED_MILLISECONDS) {
        manualAdjustmentOfTheStirringOutputs();
        lastManualStirringAdjustmentTime = millis();
        toggle();
    }

    if(millis() - lastManualHeatingAdjustmentTime >= ONE_SECOND) {
        manualAdjustmentOfTheHeatingOutputs();
        lastManualHeatingAdjustmentTime = millis();
    }

    if(millis() - lastSendHMIManualAdjustmentMeasurementsTime >= ONE_SECOND) {
        sendHMIManualAdjustmentMeasurements();
        lastSendHMIManualAdjustmentMeasurementsTime = millis();
    }

}

void MultiHeaterStirrerController::manualAdjustmentOfTheStirringOutputs()
{
    uint8_t pwmValue;
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        pwmValue = map( readADC(STIRRING_ADC_CHANNELS[i], STIRRING_KNOBS), 
                            MIN_ADC_VALUE, 
                            MAX_ADC_VALUE, 
                            MIN_LIMIT_PWM_VALUE, 
                            MAX_LIMIT_PWM_VALUE);
        stirringControllers.at(i).updateInput();
        stirringControllers.at(i).adjustOutputSignalManually(pwmValue);
    }
}

void MultiHeaterStirrerController::manualAdjustmentOfTheHeatingOutputs()
{
    uint8_t semicyclesValue;
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        semicyclesValue = map( readADC(HEATING_ADC_CHANNELS[i], HEATING_KNOBS), 
                            MIN_ADC_VALUE, 
                            MAX_ADC_VALUE, 
                            MIN_LIMIT_SEMICYCLE_VALUE, 
                            MAX_LIMIT_SEMICYCLE_VALUE);

        heatingControllers.at(i).adjustOutputSignalManually(semicyclesValue);
    }
}

void MultiHeaterStirrerController::sendHMIManualAdjustmentMeasurements()
{
    manualAdjustmentMeasurements.infraredSensorTemp = readADC(INFRAREF_HEATING_CHANNEL_SENSOR, ADC_INFRARRED_SENSOR);
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        manualAdjustmentMeasurements.RPM[i] = stirringControllers.at(i).getInput();
        manualAdjustmentMeasurements.temperatures[i] = heatingControllers.at(i).getTemperature();
    }
    RemoteCommunication_2::sendManualAdjustmentMeasurements(manualAdjustmentMeasurements);
    
}

void MultiHeaterStirrerController::generateRampSetponts()
{
    float initialTemp = RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].initialTemperature;
    float finalTemp = RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[currentProcess].finalTemperature;
    float duration = RemoteCommunication_2::processesSpecificationsMessage.processDuration[currentProcess] * 60;
    float increment = (finalTemp - initialTemp) / duration;
    rampSetpoints.clear();
    for (int i = 0; i <= duration; i++) {
        rampSetpoints.push_back(initialTemp + increment * i);
    }

    /*
    for (int i = 0; i < rampSetpoints.size(); i++) {
        Serial.print("Ramp Setpoint ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(rampSetpoints[i]);
    }
    */
}

OperationMode MultiHeaterStirrerController::readOperationModeButton()
{
    if(digitalRead(operationModeButtonPin) == HIGH) {
        return OperationMode::Manual;
    } else {
        return OperationMode::Automatic;
    }
}

uint32_t MultiHeaterStirrerController::readADC(uint8_t channel, byte adc)
{
    uint8_t data[SPI_TRANSFER_LEGNTH];
    data[0] = 0b00000001;
    data[1] = 0b10000000 | (channel << 4);
    SPI.beginTransaction(SPISettings(MAX_SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE0));
    MCP23017::digitalWrite(adc, LOW);
    for (size_t i = 0; i < SPI_TRANSFER_LEGNTH; ++i)
    {
    data[i] = SPI.transfer(data[i]);
    }

    SPI.endTransaction();
    MCP23017::digitalWrite(adc, HIGH);
    return ((data[SPI_TRANSFER_LEGNTH - 2] << 8) | data[SPI_TRANSFER_LEGNTH - 1]) & BIT_MASK;
}

void MultiHeaterStirrerController::toggle()
{
    if(blink) {
    MCP23017::digitalWrite(LED, HIGH);
    } else {
    MCP23017::digitalWrite(LED, LOW);
    }
    blink = !blink;
}


void MultiHeaterStirrerController::prueba()
{
    delay(1000);
    heatingControllers.at(0).adjustOutputSignalManually(120);
    Serial.println(heatingControllers.at(0).getTemperature());
    toggle();
}


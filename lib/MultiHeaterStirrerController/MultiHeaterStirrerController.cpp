#include "MultiHeaterStirrerController.h"

MultiHeaterStirrerController::MultiHeaterStirrerController()
{
    
}


void MultiHeaterStirrerController::setupMultiHeaterStirrerController(void (*interruptFunctions[])(void))
{
    adc1.begin(ADC_1_PIN);
    adc2.begin(ADC_2_PIN);

    automaticProcessStatus = AutomaticProcessStatus::PendingDataSubmission;
    for (int i = 0; i < NUMBER_OF_PLACES; ++i) {
        heatingControllers.emplace_back(HeatingController(  heatingKp[i], 
                                                            heatingKi[i],  
                                                            heatingKd[i], 
                                                            csMAX6675[i]));
        stirringControllers.emplace_back(StirringController(  MOTOR_A_PINS[i], 
                                                            ENCODER_PHASE_A_PINS[i], 
                                                            i, 
                                                            PULSES_PER_REVOLUTION, 
                                                            stirringKp[i],
                                                            stirringKi[i],
                                                            stirringKd[i]));
    }

    for(uint8_t i = 0; i > NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).configureStirringControllerPins(FRECUENCY, PWM_RESOLUTION, interruptFunctions[i]);
        stirringControllers.at(i).setOutputLimits((double)MIN_LIMIT_PWM_VALUE, (double)MAX_LIMIT_PWM_VALUE);
        stirringControllers.at(i).setSetpoint(0);
    }
    pinMode(operationModeButtonPin, INPUT);
    RemoteCommunication_2::beginRemoteCommunication();
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);

    lastManualStirringAdjustmentTime = millis();
    lastManualPrintTime = millis();
}

void MultiHeaterStirrerController::mainLoop()
{
    if(readOperationModeButton() == OperationMode::Automatic) {
        setControllersMode(ControlMode::Automatic);
        automaticProcess();
    } else if(readOperationModeButton() == OperationMode::Manual) {
        setControllersMode(ControlMode::Manual);
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

    Serial.println("processDuration: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[i]);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.println(RemoteCommunication_2::processesSpecificationsMessage.configuredProcesses);
    
    automaticProcessStatus = AutomaticProcessStatus::AutomaticProcessInProgress;
}

void MultiHeaterStirrerController::AutomaticProcessInProgress()
{
    
}

void MultiHeaterStirrerController::manualProcess()
{
    if(millis() - lastManualStirringAdjustmentTime >= 200) {
        manualAdjustmentOfTheStirringOutputs();
        lastManualStirringAdjustmentTime = millis();
    }

}

void MultiHeaterStirrerController::manualAdjustmentOfTheStirringOutputs()
{
    uint32_t analogReads[NUMBER_OF_PLACES];
    uint8_t pwmValues[NUMBER_OF_PLACES];
    for(uint8_t i = 0; i < NUMBER_OF_ADC_1_CHANNELS; ++i) {
        analogReads[i] = adc1.analogRead(STIRRING_ADC_1_CHANNELS[i]);
        pwmValues[i] = map(analogReads[i], MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_LIMIT_PWM_VALUE, MAX_LIMIT_PWM_VALUE);
    }

    for(uint8_t i = 0; i < NUMBER_OF_ADC_2_CHANNELS; ++i) {
        analogReads[i + NUMBER_OF_ADC_1_CHANNELS] = adc2.analogRead(STIRRING_ADC_2_CHANNELS[i]);
        pwmValues[i + + NUMBER_OF_ADC_1_CHANNELS] = map(analogReads[i + NUMBER_OF_ADC_1_CHANNELS], MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_LIMIT_PWM_VALUE, MAX_LIMIT_PWM_VALUE);
    }
    
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).adjustOutputSignalManually(pwmValues[i]);
    }

}

void MultiHeaterStirrerController::setControllersMode(ControlMode _controlMode)
{
    for(uint8_t i = 0; i > NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).setControlMode(_controlMode);
        heatingControllers.at(i).setControlMode(_controlMode);
    }
}

OperationMode MultiHeaterStirrerController::readOperationModeButton()
{
    if(digitalRead(operationModeButtonPin) == HIGH) {
        return OperationMode::Manual;
    } else {
        return OperationMode::Automatic;
    }
}



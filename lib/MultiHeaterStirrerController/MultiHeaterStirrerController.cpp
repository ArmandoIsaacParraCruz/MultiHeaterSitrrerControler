#include "MultiHeaterStirrerController.h"

MultiHeaterStirrerController::MultiHeaterStirrerController()
{
    
}


void MultiHeaterStirrerController::setupMultiHeaterStirrerController(void (*interruptFunctions[])(void))
{
    adc1.begin(ADC_1_PIN);
    adc2.begin(ADC_2_PIN);

    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        pinMode(CS_MAX6675[i], OUTPUT);
        digitalWrite(CS_MAX6675[i], HIGH);
    }

    automaticProcessStatus = AutomaticProcessStatus::PendingDataSubmission;
    for (int i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.emplace_back(StirringController(    MOTOR_A_PINS[i], 
                                                                ENCODER_PHASE_A_PINS[i], 
                                                                i, 
                                                                PULSES_PER_REVOLUTION,
                                                                FRECUENCY,
                                                                PWM_RESOLUTION,
                                                                interruptFunctions[i], 
                                                                stirringKp[i],
                                                                stirringKi[i],
                                                                stirringKd[i]));
    }

    for(uint8_t i = 0; i > NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).setOutputLimits(0, 255);
        stirringControllers.at(i).setSetpoint(0);
    }

    pinMode(ENCODER_PHASE_A_PINS[0], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[0], interruptFunctions[0], RISING);
    pinMode(ENCODER_PHASE_A_PINS[1], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[1], interruptFunctions[1], RISING);
    pinMode(ENCODER_PHASE_A_PINS[2], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[2], interruptFunctions[2], RISING);
    pinMode(ENCODER_PHASE_A_PINS[3], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[3], interruptFunctions[3], RISING);
    pinMode(ENCODER_PHASE_A_PINS[4], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[4], interruptFunctions[4], RISING);
    pinMode(ENCODER_PHASE_A_PINS[5], INPUT_PULLUP);
    attachInterrupt(ENCODER_PHASE_A_PINS[5], interruptFunctions[5], RISING);



    pinMode(operationModeButtonPin, INPUT);
    RemoteCommunication_2::beginRemoteCommunication();
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);

    lastManualStirringAdjustmentTime = millis();
    lastManualPrintTime = millis();
    Serial.println("READY_SETUP");
}

void MultiHeaterStirrerController::mainLoop()
{
    if(readOperationModeButton() == OperationMode::Automatic) {
        automaticProcess();
    } else if(readOperationModeButton() == OperationMode::Manual) {
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

    if(millis() - lastManualPrintTime >= 1000) {
        for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
            Serial.print(pwmValues[i]);
            Serial.print("-");
            Serial.print(stirringControllers.at(i).getOutput());
            Serial.print(":");
            Serial.print(stirringControllers.at(i).getInput());
            Serial.print("     ");
        }
        Serial.println();
        lastManualPrintTime = millis();
    }

}

void MultiHeaterStirrerController::manualAdjustmentOfTheStirringOutputs()
{
    for(uint8_t i = 0; i < NUMBER_OF_ADC_1_CHANNELS; ++i) {
        analogReads[i] = adc1.analogRead(STIRRING_ADC_1_CHANNELS[i]);
        pwmValues[i] = map(analogReads[i], MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_LIMIT_PWM_VALUE, MAX_LIMIT_PWM_VALUE);
    }

    for(uint8_t i = 0; i < NUMBER_OF_ADC_2_CHANNELS; ++i) {
        analogReads[i + NUMBER_OF_ADC_1_CHANNELS] = adc2.analogRead(STIRRING_ADC_2_CHANNELS[i]);
        pwmValues[i + + NUMBER_OF_ADC_1_CHANNELS] = map(    analogReads[i + NUMBER_OF_ADC_1_CHANNELS], 
                                                            MIN_ADC_VALUE, MAX_ADC_VALUE, 
                                                            MIN_LIMIT_PWM_VALUE, 
                                                            MAX_LIMIT_PWM_VALUE);
    }
    
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).adjustOutputSignalManually(pwmValues[i]);
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



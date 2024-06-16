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

        heatingControllers.emplace_back(HeatingController(  heatingKp[i], 
                                                            heatingKi[i], 
                                                            heatingKd[i], 
                                                            CS_MAX6675[i]));
    }

    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).setOutputLimits(0.0, 255.0);
        stirringControllers.at(i).setSetpoint(0);
    }

    HeatingController::setHeatingManagerPins(CS_POWER_HEATING_MANAGER, RESET_POWER_HEATING_MANAGER);

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
    operationMode = readOperationModeButton();
    RemoteCommunication_2::beginRemoteCommunication();
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);

    lastManualStirringAdjustmentTime = millis();
    lastManualHeatingAdjustmentTime = millis();
    lastSendHMIManualAdjustmentMeasurementsTime = millis();
    Serial.println("READY_SETUP");
}

void MultiHeaterStirrerController::resetSettings()
{
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).adjustOutputSignalManually(0);
    }
    automaticProcessStatus = AutomaticProcessStatus::PendingDataSubmission;
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(ProcessMessageStatus::NotReceived);
    HeatingController::resetHeatingManager();
}

void MultiHeaterStirrerController::mainLoop()
{
    if(operationMode != readOperationModeButton()) {
        HeatingController::resetHeatingManager();
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

    Serial.println("processDuration: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[i]);
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
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        if(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]) {
            Serial.print(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[currentProcess]);
            Serial.print(" ");
            stirringControllers.at(i).setSetpoint(RemoteCommunication_2::processesSpecificationsMessage.stirringSetpoints[currentProcess]);
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
        Serial.println();
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
           stirringControllers.at(i).adjustOutputSignal();
        }
    }
}

void MultiHeaterStirrerController::produceHeatingPIDOutputs()
{

}


void MultiHeaterStirrerController::sendHMIAutomaticProcessesMeasurements()
{
    measurements.infraredSensorTemp = adc2.analogRead(INFRAREF_HEATING_CHANNEL_SENSOR);
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        measurements.RPM[i] = stirringControllers.at(i).getInput();
        measurements.temperatures[i] = heatingControllers.at(i).getTemperature();
    }
    measurements.timeInSencods = (millis() - lastAutomaticProcessTime) / 1000;
    RemoteCommunication_2::sendMeasurements(measurements);
}


void MultiHeaterStirrerController::manualProcess()
{
    if(millis() - lastManualStirringAdjustmentTime >= TWO_HUNDRED_MILLISECONDS) {
        manualAdjustmentOfTheStirringOutputs();
        Serial.println(stirringControllers.at(0).getInput());
        lastManualStirringAdjustmentTime = millis();
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
    for(uint8_t i = 0; i < NUMBER_OF_ADC_1_CHANNELS; ++i) {
        analogReads[i] = adc1.analogRead(STIRRING_ADC_1_CHANNELS[i]);
        pwmValues[i] = map(analogReads[i], MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_LIMIT_PWM_VALUE, MAX_LIMIT_PWM_VALUE);
    }

    for(uint8_t i = 0; i < NUMBER_OF_ADC_2_CHANNELS; ++i) {
        analogReads[i + NUMBER_OF_ADC_1_CHANNELS] = adc2.analogRead(STIRRING_ADC_2_CHANNELS[i]);
        pwmValues[i + NUMBER_OF_ADC_1_CHANNELS] = map(    analogReads[i + NUMBER_OF_ADC_1_CHANNELS], 
                                                            MIN_ADC_VALUE, MAX_ADC_VALUE, 
                                                            MIN_LIMIT_PWM_VALUE, 
                                                            MAX_LIMIT_PWM_VALUE);
    }
    
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        stirringControllers.at(i).adjustOutputSignalManually(pwmValues[i]);
    }

}

void MultiHeaterStirrerController::manualAdjustmentOfTheHeatingOutputs()
{
    for(uint8_t i = 0; i < NUMBER_OF_ADC_1_CHANNELS; ++i) {
        analogReads[i] = adc1.analogRead(HEATING_ADC_1_CHANNELS[i]);
        semicyclesValues[i] = map(analogReads[i], MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_LIMIT_SEMICYCLE_VALUE, MAX_LIMIT_SEMICYCLE_VALUE);
    }

    for(uint8_t i = 0; i < NUMBER_OF_ADC_2_CHANNELS; ++i) {
        analogReads[i + NUMBER_OF_ADC_1_CHANNELS] = adc2.analogRead(HEATING_ADC_2_CHANNELS[i]);
        semicyclesValues[i + NUMBER_OF_ADC_1_CHANNELS] = map(    analogReads[i + NUMBER_OF_ADC_1_CHANNELS], 
                                                                MIN_ADC_VALUE, MAX_ADC_VALUE, 
                                                                MIN_LIMIT_SEMICYCLE_VALUE, 
                                                                MAX_LIMIT_SEMICYCLE_VALUE);
    }
    
    HeatingController::transferSemicycles(semicyclesValues);
}

void MultiHeaterStirrerController::sendHMIManualAdjustmentMeasurements()
{
    manualAdjustmentMeasurements.infraredSensorTemp = adc2.analogRead(INFRAREF_HEATING_CHANNEL_SENSOR);
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        manualAdjustmentMeasurements.RPM[i] = stirringControllers.at(i).getInput();
        manualAdjustmentMeasurements.temperatures[i] = heatingControllers.at(i).getTemperature();
    }
    RemoteCommunication_2::sendManualAdjustmentMeasurements(manualAdjustmentMeasurements);
}


OperationMode MultiHeaterStirrerController::readOperationModeButton()
{
    if(digitalRead(operationModeButtonPin) == HIGH) {
        return OperationMode::Manual;
    } else {
        return OperationMode::Automatic;
    }
}



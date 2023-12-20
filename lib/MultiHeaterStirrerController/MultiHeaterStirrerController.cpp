#include "MultiHeaterStirrerController.h"

MultiHeaterStirrerController::MultiHeaterStirrerController()
{
   
}

void MultiHeaterStirrerController::setupMultiHeaterStirrerController()
{
    pinMode(operationModeButtonPin, INPUT);
    RemoteCommunication_2::beginRemoteCommunication();
    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(notReceived);
    
}

void MultiHeaterStirrerController::mainLoop()
{
    
    while (true)
    {
        if(readOperationModeButton() == Automatic) {
            Serial.println("Automatic");
            waitForProcessesSpecificationsMessage();
        } else {
            Serial.println("Manual");
        }
    }
}

void MultiHeaterStirrerController::waitForProcessesSpecificationsMessage()
{
    while (RemoteCommunication_2::getProcessesSpecificationsMessageStatus() == notReceived)
    {
        if(readOperationModeButton() == Manual) 
        {
            return;
        }
    }

    RemoteCommunication_2::setProcessesSpecificationsMessageStatus(notReceived);
    
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.selectedPlaces[i]);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.println("temperatureSetpoints: ");
    for(uint8_t i = 0; i < NUMBER_OF_PROCESS; ++i) {
        Serial.print("Inicial: ");
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[i].finalTemperature);
        Serial.print("  Final: ");
        Serial.print(RemoteCommunication_2::processesSpecificationsMessage.temperatureSetpoints[i].initialTemperature);
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
       
}

OperationMode MultiHeaterStirrerController::readOperationModeButton()
{
    if(digitalRead(operationModeButtonPin) == HIGH) {
        return Manual;
    } else {
        return Automatic;
    }
}

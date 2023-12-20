#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <MCP3XXX.h>
#include "HeatingController.h"
#include "StirringController.h"
#include "RemoteCommunication_2.h"
#include "StructureMessages.h"

enum OperationMode{Manual, Automatic};


class MultiHeaterStirrerController
{
    public:
        MultiHeaterStirrerController();
        void setupMultiHeaterStirrerController();
        void mainLoop();
    private:
        const uint8_t operationModeButtonPin = 8;
        //HeatingController heatingControllers[NUMBER_OF_PLACES];
        //StirringController stirringControllers[NUMBER_OF_PLACES];
        uint8_t heatingResistorPin[NUMBER_OF_PLACES];
        uint8_t clkPin[NUMBER_OF_PLACES];
        void waitForProcessesSpecificationsMessage();
        OperationMode readOperationModeButton();
        
};

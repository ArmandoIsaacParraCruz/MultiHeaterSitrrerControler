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
        std::vector<HeatingController> heatingControllers;
        //StirringController stirringControllers;
        const double heatingKp[NUMBER_OF_PLACES] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        const double heatingKi[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        const double heatingKd[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
         const uint8_t csMAX6675[NUMBER_OF_PLACES] = {6, 7, 15, 16, 17, 18};
        void waitForProcessesSpecificationsMessage();
        OperationMode readOperationModeButton();
        void testLoop();
};

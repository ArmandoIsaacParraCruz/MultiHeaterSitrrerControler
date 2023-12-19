#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <MCP3XXX.h>
#include "HeatingController.h"
#include "StirringController.h"
#include "RemoteCommunication.h"
#include "StructureMessages.h"




class MultiHeaterStirrerController
{
    public:
        MultiHeaterStirrerController();
        void mainLoop();

    private:
        //HeatingController heatingControllers[NUMBER_OF_PLACES];
        //StirringController stirringControllers[NUMBER_OF_PLACES];
        uint8_t heatingResistorPin[NUMBER_OF_PLACES];
        uint8_t clkPin[NUMBER_OF_PLACES];
        
};

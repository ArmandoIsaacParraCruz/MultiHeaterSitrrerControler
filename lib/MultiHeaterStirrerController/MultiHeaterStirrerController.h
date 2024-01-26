#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <MCP3XXX.h>
#include "HeatingController.h"
#include "StirringController.h"
#include "RemoteCommunication_2.h"
#include "StructureMessages.h"

enum class OperationMode {Manual, Automatic };

enum class AutomaticProcessStatus {
    PendingDataSubmission,
    ProcessingDataReceived,
    AutomaticProcessInProgress
};


class MultiHeaterStirrerController
{
    public:
        MultiHeaterStirrerController();
        void setupMultiHeaterStirrerController(void (*interruptFunctions[])(void));
        void mainLoop();

        std::vector<StirringController> stirringControllers;

    private:
        MCP3008 adc1;
        MCP3008 adc2;
        const uint8_t ADC_1_PIN = 4;
        const uint8_t ADC_2_PIN = 5;
        static const uint8_t NUMBER_OF_ADC_1_CHANNELS = 4; 
        static const uint8_t NUMBER_OF_ADC_2_CHANNELS = 2;
        const uint8_t STIRRING_ADC_1_CHANNELS[NUMBER_OF_ADC_1_CHANNELS] = {1, 3 ,5 ,7};
        const uint8_t STIRRING_ADC_2_CHANNELS[NUMBER_OF_ADC_2_CHANNELS] = {1, 3};
        const uint8_t HEATING_ADC_1_CHANNELS[NUMBER_OF_ADC_1_CHANNELS] = {0, 2, 4, 6};
        const uint8_t HEATING_ADC_2_CHANNELS[NUMBER_OF_ADC_2_CHANNELS] = {0, 2};
        const uint8_t CS_MAX6675[NUMBER_OF_PLACES] = {6, 7, 15, 16, 17, 18};
        const uint8_t INFRAREF_HEATING_CHANNEL_SENSOR = 4;
        const uint8_t MIN_ADC_VALUE = 0;
        const uint16_t MAX_ADC_VALUE = 1023;
        const uint8_t MIN_LIMIT_PWM_VALUE = 0;
        const uint8_t MAX_LIMIT_PWM_VALUE = 255;
        AutomaticProcessStatus automaticProcessStatus;
        const uint8_t operationModeButtonPin = 8;
        std::vector<HeatingController> heatingControllers;
        //StirringController stirringControllers;
        const double heatingKp[NUMBER_OF_PLACES] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        const double heatingKi[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        const double heatingKd[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        const double stirringKp[NUMBER_OF_PLACES] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        const double stirringKi[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        const double stirringKd[NUMBER_OF_PLACES] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        const uint8_t csMAX6675[NUMBER_OF_PLACES] = {6, 7, 15, 16, 17, 18};
        const uint8_t MOTOR_A_PINS[NUMBER_OF_PLACES] = {37, 36, 35, 47, 21, 20};
        const uint8_t ENCODER_PHASE_A_PINS[NUMBER_OF_PLACES] = {1, 2, 42, 41, 40, 39};
        const float PULSES_PER_REVOLUTION = 48.4;
        const uint32_t FRECUENCY = 15000;
        const uint8_t PWM_RESOLUTION = 8; 
        uint32_t lastManualStirringAdjustmentTime;
        uint32_t lastManualPrintTime;
        uint32_t analogReads[NUMBER_OF_PLACES];
        uint8_t pwmValues[NUMBER_OF_PLACES];

        void automaticProcess();
        void PendingDataSubmission();
        void ProcessingDataReceived();
        void AutomaticProcessInProgress();
        void manualProcess();  
        void manualAdjustmentOfTheStirringOutputs(); 
        OperationMode readOperationModeButton();
        void testLoop();
};






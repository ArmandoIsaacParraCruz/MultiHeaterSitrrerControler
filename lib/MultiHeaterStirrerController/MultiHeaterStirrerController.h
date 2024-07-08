#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include "HeatingController.h"
#include "StirringController.h"
#include "RemoteCommunication_2.h"
#include "StructureMessages.h"
#include "MCP23017.h"

#define MCP23017_ADDRESS 0x20
#define ONE_SECOND 1000
#define TWO_HUNDRED_MILLISECONDS 200
#define SDA 8
#define SCL 18
#define LED 12
#define ZERO_CROSSING_DEBOUNCE_TIME 8
#define MAX_SPI_CLOCK_SPEED 1350000
#define BIT_MASK 1023
#define SPI_TRANSFER_LEGNTH  3
#define STIRRING_KNOBS 9
#define HEATING_KNOBS 8
#define ADC_INFRARRED_SENSOR 8

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
        void prueba();
        std::vector<StirringController> stirringControllers;
        static uint32_t zeroCrossingDebounceTimer;
        std::vector<HeatingController> heatingControllers;
        const uint8_t MIN_LIMIT_SEMICYCLE_VALUE = 0;
        const uint8_t MAX_LIMIT_SEMICYCLE_VALUE = 120;
    private:
        Measurements measurements;
        ManualAdjustmentMeasurements manualAdjustmentMeasurements;
        OperationMode operationMode;
        AutomaticProcessStatus automaticProcessStatus;
        bool blink = true;

        std::vector<float>rampSetpoints;

        const uint8_t STIRRING_ADC_CHANNELS[NUMBER_OF_PLACES] = {0, 1, 2 ,3 ,4, 5};
        const uint8_t HEATING_ADC_CHANNELS[NUMBER_OF_PLACES] = {2, 3, 4, 5, 6, 7};
        const uint8_t CS_MAX6675[NUMBER_OF_PLACES] = {0, 1, 2, 3, 4, 5};
        const uint8_t INFRAREF_HEATING_CHANNEL_SENSOR = 0;
        const uint8_t MIN_ADC_VALUE = 0;
        const uint16_t MAX_ADC_VALUE = 1023;
        const uint8_t MIN_LIMIT_PWM_VALUE = 0;
        const uint8_t MAX_LIMIT_PWM_VALUE = 255;
        const uint8_t operationModeButtonPin = 9;
        //StirringController stirringControllers;
        
        const double heatingKp[NUMBER_OF_PLACES] = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
        const double heatingKi[NUMBER_OF_PLACES] = {0.011, 0.011, 0.011, 0.011, 0.011, 0.011};
        const double heatingKd[NUMBER_OF_PLACES] = {3, 3, 3, 3, 3, 3};
        const double alpha[NUMBER_OF_PLACES] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        const double beta[NUMBER_OF_PLACES] = {1.7, 1.7, 1.7, 1.7, 1.7, 1.7};
        const double tau1[NUMBER_OF_PLACES] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        const double tau2[NUMBER_OF_PLACES] = {1.7, 1.7, 1.7, 1.7, 1.7, 1.7};

        const uint8_t RESISTORS_PINS[NUMBER_OF_PLACES] = {4, 5, 6, 7, 15, 16};
        const uint8_t ZERO_CROSSING_PIN = 14;

        const double stirringKp[NUMBER_OF_PLACES] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        const double stirringKi[NUMBER_OF_PLACES] = {0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625};
        const double stirringKd[NUMBER_OF_PLACES] = {1, 1, 1, 1, 1, 1};
        
        const uint8_t MOTOR_A_PINS[NUMBER_OF_PLACES] = {20, 21, 47, 35, 36, 37};
        const uint8_t ENCODER_PHASE_A_PINS[NUMBER_OF_PLACES] = {1, 2, 42, 41, 40, 39};
        const float PULSES_PER_REVOLUTION = 70;
        const uint32_t FRECUENCY = 1000;
        const uint8_t PWM_RESOLUTION = 8; 

        uint32_t lastAutomaticStirringAdjustmentTime;
        uint32_t lastManualStirringAdjustmentTime;
        uint32_t lastAutomaticHeatingAdjustmentTime;
        uint32_t lastManualHeatingAdjustmentTime;
        uint32_t lastSendHMIManualAdjustmentMeasurementsTime;
        uint32_t lastSendHMIAutomaticAdjustmentMeasurementsTime;
        uint32_t lastAutomaticProcessTime;

        uint8_t currentProcess;
        

        void resetSettings();
        void automaticProcess();
        void PendingDataSubmission();
        void ProcessingDataReceived();
        void AutomaticProcessInProgress();
        void updatingSetponts();
        void evaluateProcessDuration();
        void produceHeatingPIDOutputs();
        void produceStirringPIDOutputs();
        void sendHMIAutomaticProcessesMeasurements();
        void manualProcess();  
        void manualAdjustmentOfTheStirringOutputs();
        void manualAdjustmentOfTheHeatingOutputs();
        void sendHMIManualAdjustmentMeasurements();
        void generateRampSetponts();
        OperationMode readOperationModeButton();
        uint32_t readADC(uint8_t channel, byte adc);
        void toggle();
};






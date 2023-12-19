#pragma once
#include <Arduino.h>

#define NUMBER_OF_PLACES        6
#define NUMBER_OF_PROCESS       15
#define MAX_TEMPERATURE         300
#define MAX_TEMPERATURE_DIGIT   999
#define MAX_RPM                 1200
#define MAX_RPM_DIGITS          9999
#define MAX_TIME                180
#define MAX_TIME_DIGITS         999  



enum TemperatureFunctionType{constant, ramp};

struct TemperatureSetpoint {
            uint16_t initialTemperature;
            uint16_t finalTemperature;
            TemperatureFunctionType tempFunction;
};

struct ProcessesSpecificationsMessage{
    bool selectedPlaces[NUMBER_OF_PLACES];
    TemperatureSetpoint temperatureSetpoints[NUMBER_OF_PROCESS];
    uint16_t stirringSetpoints[NUMBER_OF_PROCESS]; 
    uint32_t processDuration[NUMBER_OF_PROCESS];
    uint8_t configuredProcesses;
};

struct measurements {
    float temperatures[NUMBER_OF_PLACES];
    float RPM[NUMBER_OF_PLACES];
    uint32_t timeInSencods;
    float infraredSensorTemp;
};


struct  manualAdjustmentParameters{
    float temperatures[NUMBER_OF_PLACES];
    float RPM[NUMBER_OF_PLACES];
    float infraredSensorTemp;
};

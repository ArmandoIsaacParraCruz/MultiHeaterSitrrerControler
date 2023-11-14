#pragma once
#include <Arduino.h>
#include <Controller.h>
#include <driver/ledc.h>
#include <esp_timer.h>


class StirringController: public Controller{
    public:
        StirringController( uint8_t _motorAPin, 
                            uint8_t _encoderPhaseAPin,
                            uint8_t _channel,
                            float _pulsesPerRevolution,
                            double _kp,
                            double _ki,
                            double _kd);

        void adjustOutputSignal() override;
        void updateInput() override;
        void incrementPulses();
        void configureStirringControllerPins(uint32_t &_frequency, 
                                            uint8_t &_resolution, 
                                            void (*interruptFunction)());
        
        ~StirringController();
       
    private:
        
        double alpha = 0.1;
        uint32_t lastMeasurementTime;
        volatile uint32_t pulses;
        uint8_t motorAPin;
        uint8_t encoderPhaseAPin;
        uint8_t channel;
        float pulsesPerRevolution;
        uint32_t debouncePreviousTime;
        const uint32_t DEBOUNCE_TIME = 500;
};

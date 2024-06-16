#pragma once
#include "Controller.h"

class StirringController: public Controller{
    public:
        StirringController(double _kp, double _ki, double _kd);

        void begin( uint8_t _motorAPin, 
                    uint8_t _encoderPhaseAPin,
                    uint8_t _channel,
                    float _pulsesPerRevolution,
                    uint32_t _frequency, 
                    uint8_t _resolution, 
                    void (*interruptFunction)());

        void adjustOutputSignal() override;
        void updateInput() override;
        void incrementPulses();
        void adjustOutputSignalManually(uint8_t _pwmValue);
        
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
        const uint32_t DEBOUNCE_TIME = 900;
};

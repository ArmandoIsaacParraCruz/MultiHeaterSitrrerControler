#pragma once
#include <Arduino.h>


class Controller
{
    protected:
        double setpoint;
        double iTerm;
        double lastInput;
        double kp;
        double ki;
        double kd;
        double outMin;
        double outMax;
        volatile double input;
        volatile double output;
        
        void computeOutput();
        
    public:
        Controller(double &_kp, double &_ki,double &_kd);
        void setSetpoint(double _newSetpoint);
        void setOutputLimits(double _min, double _max);
        double getInput();
        double getOutput();
        virtual void adjustOutputSignal() = 0;
        virtual void updateInput() = 0;
        virtual ~Controller() {}
};
#include "Controller.h"

Controller::Controller(double &_kp, double &_ki,double &_kd)
    :kp(_kp), ki(_ki), kd(_kd), lastInput(0.0), iTerm(0.0), outMin(0.0), outMax(0.0)
{
   input = 0;
}


void Controller::computeOutput()
{

    double error = setpoint - input;
   
    iTerm += (ki * error);
    if(iTerm > outMax){
        iTerm = outMax;
    } else if (iTerm < outMin){
        iTerm = outMin;
    }

    double dInput = (input - lastInput);
    output = kp * error + iTerm - kd * dInput;
    if(output > outMax) {
        output = outMax;
    } else if(output < outMin) {
        output = outMin;
    }
     
    lastInput = input;
}

void Controller::setSetpoint(double _newSetpoint)
{
    setpoint = _newSetpoint;
}

void Controller::setOutputLimits(double _min, double _max)
{
    if(_min < _max){
        outMin = _min;
        outMax = _max;
    }
}

double Controller::getInput()
{
    return input;
}

double Controller::getOutput()
{
    return output;
}

void Controller::reset()
{
    lastInput = 0.0;
    iTerm = 0.0;
    output = 0.0;
    input = 0.0;
}
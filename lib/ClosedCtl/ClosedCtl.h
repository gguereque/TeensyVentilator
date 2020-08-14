
#ifndef ClosedCtl_h
#define ClosedCtl_h

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "Interface.h"


struct Sensor{
    Feedback sensorType;
    float minVal;
    float maxVal;
    float currentValue;
};

struct Pid_params{
   double kp;
   double ki;
   double kd;
   double setPoint;
};
struct MotorParams{
    float minVal;
    float maxVal;
};

class ClosedCtl{
public:
    Sensor sensor;
    Pid_params pid_params;
    MotorParams motor_params;

   
private:
    long _interval;
    double _kp,_ki,_kd;

};

#endif //ClosedCtl_h
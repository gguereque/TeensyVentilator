#ifndef serialCommands_h
#define serialCommands_h

#define PULSES_REV 400.0

#include <Arduino.h>
#include "VentCtl.h"
#include <EEPROM.h>
#include "TeensyStep.h"


// output pins for motion control on 3 steppers
constexpr int stpPin_in = 2, dirPin_in = 3;
constexpr int stpPin_out = 4, dirPin_out = 6;
constexpr int stpPin_o2 = 11, dirPin_o2 = 12;



void handleManualCommands(VentCtl*,String,int,int);
void handleCommandValues(VentCtl*,String,String,float);
void getSerialParams(VentCtl*,String,bool,bool,short);
void moveMotor(double,int);
void initMotors();
void setAllMotorHome();
void setMotorHome(int);
int getPosition(int);
void setPosition(int,int);
#endif
#ifndef processStates_h
#define processStates_h

#define REDUCTION_O2 5.18
#define SAMPLE_TIME 1

#include "serialCommands.h"
#include <PID_v1.h>
#include <pidautotuner.h>

enum TestMode{
    TUNE,
    INIT,
    EXEC
};

void graphPid();
void homeAll(VentCtl *,int);
void stop(VentCtl *);
void selectCtl(VentCtl*,TestMode);
void autoTunePID(VentCtl*,float,long,ClosedCtl);
void InitMainPID(VentCtl*, ClosedCtl);
void InitAuxPID(VentCtl*, ClosedCtl);
void HandleMainPID(VentCtl*, ClosedCtl);
void HandleAuxPID(VentCtl*, ClosedCtl);
void handleInhaleAux(VentCtl*);
void handleExhaleAux(VentCtl*);
void selectMotorMove(Sensor,double,bool);
double selectSensorValue(VentCtl*,Sensor);
void  getValues(VentCtl*);
void initPidTime();
void initAux();
bool getAuxStatus();
void checkForAux(VentCtl *);
void getCountCycles(VentCtl *);


#endif
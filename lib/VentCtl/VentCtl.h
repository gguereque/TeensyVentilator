
#ifndef VentCtl_h
#define VentCtl_h

#define AIR_DENSITY 1.20
#define AREA_2 0.00009698698
#define AREA_1 0.0003800306


#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "ClosedCtl.h"
#include "Omron_D6FPH.h"
#include "Honeywell_HSC_SSC.h"



  enum States{
      HOMING,
      HOMED,
      STARTING,
      STARTED,
      INIT_INHALING,
      INHALING,
      INIT_EXHALING,
      EXHALING,
      TUNNING,
      TUNED,
      INIT_PID_TEST,
      PID_TESTING,
      STOPPING,
      STOPPED,
      IDLE,
      ERROR

  };
  // enum sensor brands
enum SensorBrand{
    OMRON,
    HONEYWELL
};

class VentCtl{
public:
    States state;
    States prev_state;
    SensorBrand sensor_brand;
    Interface iface;
    Omron_D6FPH *flowSensor_in,*flowSensor_out;
    ClosedCtl pressure_in,pressure_out,flow_in,flow_out,o2;
    char* stateStr[16] = {"HOMING","HOMED", "STARTING", "STARTED", "INIT_INHALING", "INHALING","INIT_EXHALING", "EXHALING",
                       "TUNNING", "TUNED", "INIT_PID_TEST", "PID_TESTING", "STOPPING", "STOPPED", "IDLE", "ERROR"};
    VentCtl();
    void Init();
    void getState(States);
    void setState(States);
    void defineFlowBrand(SensorBrand);
    // Ctl functions
    double getI2CInput(Feedback,float,float);
    float getFlow(int);
private:
// Air flow constant for further calculations
  double kf = 0.0;

};

#endif //VentCtl_h
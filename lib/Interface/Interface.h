
#ifndef Interface_h
#define Interface_h

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

  enum ControlMode {
    FLOW_CTL,
    PRESSURE_CTL,
    NONE,
  };

  enum OperationMode {
      CTL,
      CTL_ASST,
      PS,
  };

  enum Feedback{
    FLOW_IN,
    FLOW_OUT,
    PRESSURE_IN,
    PRESSURE_OUT,
    O2
  };

class Interface {
public:

struct cfg_params{
    float FR;
    float FIO2;
    float PEEP;
    float TV;            
    float IF;       
    float TI;
    float PINS;
    float R;
    unsigned long testTime;    
}pms;

enum StateAlarm
{
    NO_RELEVANT,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

enum AlarmType{
  HIGH_PRESSURE_ENTRY,
  HIGH_O2_ENTRY,
  LOW_PRESSURE_ENTRY,
  LOW_O2_ENTRY,
  FLOW_IN_HIGH,
  FLOW_IN_LOW,
};

struct Alarm{
  StateAlarm priority;
  AlarmType type;
  char * message;
};

  OperationMode omode;
  ControlMode cmode;
  Feedback atune;
  // getter functions
  float TC() const {return 60.0 / pms.FR;}
  uint16_t TE() const {return TC() - pms.TI;}
  float C() const {return pms.TV / pms.PINS;}

private:

};

#endif //Interface_h
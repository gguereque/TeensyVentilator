#ifndef OMROND6FPH_H
#define OMROND6FPH_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define D6FPH_ADDRESS   0x6C
enum sensorModels{
    MODEL_0025AD1 = 0,
    MODEL_0505AD3,
    MODEL_5050AD3
};

class OmronD6FPH
{
public:
	OmronD6FPH(void);
    void initialize(uint8_t);
    int flow(int);
    int temperature();

private:
    uint8_t _address;
};

#endif
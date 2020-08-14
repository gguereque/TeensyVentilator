#include "VentCtl.h"
#include <pidautotuner.h>
#include "Honeywell_HSC_SSC.h"


#define PULSES_REV 400.0




 VentCtl::VentCtl(){
 pressure_in.sensor.sensorType = PRESSURE_IN;
 pressure_out.sensor.sensorType = PRESSURE_OUT;
 flow_in.sensor.sensorType = FLOW_IN;
 flow_out.sensor.sensorType = FLOW_OUT;
 o2.sensor.sensorType = O2;

 }
 void VentCtl::defineFlowBrand(SensorBrand sensor){
     sensor_brand = sensor;
 }


void VentCtl::Init(){
 pressure_in.sensor.sensorType = PRESSURE_IN;
 pressure_out.sensor.sensorType = PRESSURE_OUT;
 flow_in.sensor.sensorType = FLOW_IN;
 flow_out.sensor.sensorType = FLOW_OUT;
 o2.sensor.sensorType = O2;  
 kf = 1 - sq(AREA_2/AREA_1);

}

double VentCtl::getI2CInput(Feedback sensortype, float minVal, float maxVal){
   byte address;
   int outMin,outMax;
   switch (sensortype)
   {
   case PRESSURE_IN: 
       address = 0x58; outMin = 0x666; outMax = 0x399A;
       break;
    case FLOW_IN: 
       address = 0x28; outMin = 0x666; outMax = 0x399A;
       break;
   
   default:
       address = 0x58; outMin = 0x666; outMax = 0x399A;
       break;
   }
  struct cs_raw ps;
    char p_str[10], t_str[10];
    uint8_t el;
    float p, t;
    el = ps_get_raw(address, &ps);
    // for some reason my chip triggers a diagnostic fault
    // on 50% of powerups without a notable impact 
    // to the output values.
    if ( el == 4 ) {
        Serial.println("err sensor.sensorType missing");
        return 0;
    } else {
        if ( el == 3 ) {
            Serial.print("err diagnostic fault ");
            Serial.println(ps.status, BIN);
        }
        if ( el == 2 ) {
            // if data has already been feched since the last
            // measurement cycle
            Serial.print("warn stale data ");
            Serial.println(ps.status, BIN);
        }
        if ( el == 1 ) {
            // chip in command mode
            // no clue how to end up here
            Serial.print("warn command mode ");
            Serial.println(ps.status, BIN);
        }
        // Serial.print("status      ");
        // Serial.println(ps.status, BIN);
        // Serial.print("bridge_data ");
        // Serial.println(ps.bridge_data, DEC);
        // Serial.print("temp_data   ");
        // Serial.println(ps.temperature_data, DEC);
        // Serial.println("");
        ps_convert(ps, &p, &t, outMin, outMax, minVal,
                maxVal);
        // floats cannot be easily printed out
        dtostrf(p, 2, 2, p_str);
        dtostrf(t, 2, 2, t_str);
       // Serial.print("pressure    (Pa) ");
        // Serial.println(p_str);
        // Serial.print("temperature (dC) ");
        // Serial.println(t_str);
        // Serial.println("");
        }
        if(p > 0){return (p-0.0025);}
        else return (p+0.0025);
}

float VentCtl::getFlow(int sensorNumber){
    float flow;
    float velocity;
    if(sensor_brand == OMRON){
    switch (sensorNumber)
    {
    case 1:
        flow = flowSensor_in->getPressure();
        break;

    case 2:
        flow = flowSensor_out->getPressure();
        break;
    
    default:
        break;
    }
    }
    else if(sensor_brand == HONEYWELL){
     switch (sensorNumber)
        {
        case 1:

            flow = this->getI2CInput(this->flow_in.sensor.sensorType,-500,500);
            break;

        case 2:
            flow = this->getI2CInput(this->flow_out.sensor.sensorType,-500,500);
            break;
        
        default:
            break;
     }
     if(2*flow/(AIR_DENSITY*kf) < 0) flow = 0;
    // else flow = AREA_2*sqrt(2*flow/(AIR_DENSITY*kf))*60000.0;
    }
    return flow;
}

void VentCtl::getState(States st){
    Serial.println("STATE "+String(this->stateStr[st]));
}
void VentCtl::setState(States st){
    this->prev_state = this->state;
    this->state = st;
    getState(st);
}



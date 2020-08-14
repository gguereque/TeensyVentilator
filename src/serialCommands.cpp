#include "serialCommands.h"

Stepper motor_in(stpPin_in, dirPin_in);
StepControl controller_in;
Stepper motor_out(stpPin_out, dirPin_out);
StepControl controller_out;
Stepper motor_o2(stpPin_o2, dirPin_o2);
StepControl controller_o2;


// handle serial command functions ------------------------------------------------------

void handleManualCommands(VentCtl *ventilator,String cmd, int device, int value){
 if(ventilator->state != IDLE){ 
     if(cmd.equals("stop")){ ventilator->setState(STOPPING);  }  
     else {
         Serial.println("System is busy...please try again later!"); 
        return;
        }
    }
 if(cmd.equals("m")) {

            if(device > 0){
                moveMotor(value,device);
            }
            else Serial.println("Please specify a device as such: $cmd dev val$ where dev & val are numeric!"); 
        }
        else if(cmd.equals("s")){
            controller_in.stopAsync();             
            Serial.println("Stopping motor");
        }
        else if(cmd.equals("h")){
            Serial.println("GO TO HOMING!");
            ventilator->setState(HOMING);            
            Serial.println("Stopping motor");
        }
        else if(cmd.equals("e")){
            controller_in.emergencyStop();
            Serial.println("Emergency Stop"); 
        }
        else if(cmd.equals("t")){
            Serial.println("Autotuning PID");
            ventilator->iface.cmode = PRESSURE_CTL;
            if(value > 0){
                switch (value)
                {
                 case 1: 
                    ventilator->pressure_in.pid_params.setPoint = 15;
                    ventilator->iface.atune = PRESSURE_IN;
                    ventilator->setState(TUNNING); 
                     break;
                 case 2:
                    ventilator->flow_in.pid_params.setPoint = 50.5; 
                    ventilator->iface.atune = FLOW_IN;
                    ventilator->setState(TUNNING); 
                  break;
                case 3: 
                    ventilator->iface.atune = PRESSURE_OUT;
                    ventilator->setState(TUNNING); 
                  break;
                case 4: 
                    ventilator->iface.atune = FLOW_OUT;
                    ventilator->setState(TUNNING); 
                  break;
                case 5: 
                    ventilator->iface.atune = O2;
                    ventilator->setState(TUNNING); 
                  break;
                 default:ventilator->iface.atune = PRESSURE_IN; break;
                } 
            }
            // ventilator->autoTunePID(ventilator->sensor,2.0,1000,motor_in);
                         
        }
        else if(cmd.equals("om")){
            if(value > 0){
                Serial.println("Entering operation mode");
                int modeNumber = value;
                // switch (modeNumber)
                // {
                // case 1:
                //     //P_timer.begin(IP_PID,100);
                //     break;
                // case 2:
                //     //F_timer.begin(IP_PID,100);
                //     break;
                
                // default:
                //     P_timer.end();
                //     F_timer.end();
                //     break;
                // }
            
            }
            else  Serial.println("Please specify a value as such: $cmd val$");           
        }
        else if(cmd.equals("rp")){
            if(value > 0){
             Serial.println("Reading pressure");
                int sensorNumber = value;
                ventilator->getI2CInput(ventilator->pressure_in.sensor.sensorType,0.0,100.0);
            }
            else  Serial.println("Please specify a value as such: $cmd val$");                         
        }
        else if(cmd.equals("rf")){
            if(value > 0){
                Serial.println("Reading flow");
                int sensorNumber = value;
                ventilator->getFlow(sensorNumber);
            }
            else  Serial.println("Please specify a value as such: $cmd val$");
        }
        else if(cmd.equals("gp")){
            if(value > 0){
             Serial.println("Graphing pressure");
                int sensorNumber = value;
                elapsedMillis interval = 0;
                while(interval < 10000){
                    ventilator->getI2CInput(ventilator->pressure_in.sensor.sensorType,0.0,100.0);
                      // Plot
                    //p.Plot(); 
                    delay(100);
                }

            }
            else  Serial.println("Please specify a value as such: $cmd val$ where val is numeric!");                         
        }
        else if(cmd.equals("gf")){
            if(value > 0){
             Serial.println("Graphing flow");
                int sensorNumber = value;
                elapsedMillis interval = 0;
                while(interval < 10000){
                    double flow = ventilator->getFlow(sensorNumber);
                    Serial.println(flow);
                      // Plot
                    //p.Plot(); 
                    delay(100);
                }
            }
            else  Serial.println("Please specify a value as such: $cmd val$ where val is numeric!"); 
        }
        else if(cmd.equals("pid")){
            if(value > 0){
             Serial.println("Running Test PID");
                ventilator->iface.pms.testTime = value*100;
               switch (device)
               {
                 case 1: 
                    ventilator->pressure_in.pid_params.setPoint = ventilator->iface.pms.PINS;
                    ventilator->iface.atune = PRESSURE_IN;
                    ventilator->setState(INIT_PID_TEST); 
                     break;
                 case 2: 
                    ventilator->iface.atune = FLOW_IN;
                    ventilator->setState(INIT_PID_TEST); 
                  break;
                case 3: 
                    ventilator->iface.atune = PRESSURE_OUT;
                    ventilator->setState(INIT_PID_TEST); 
                  break;
                case 4: 
                    ventilator->iface.atune = FLOW_OUT;
                    ventilator->setState(INIT_PID_TEST); 
                  break;
                case 5: 
                    ventilator->iface.atune = O2;
                    ventilator->setState(INIT_PID_TEST); 
                  break;
                 default:ventilator->iface.atune = PRESSURE_IN; break;
                
               }
            }
            else  Serial.println("Please specify a value as such: $cmd val$ where val is numeric!"); 
         } 
        else if(cmd.equals("start")){ ventilator->setState(STARTING);  }                      
        else if(cmd.equals("help") || cmd.equals("u")){
            Serial.println("\nUsage:");
            Serial.println("  m: move motor");
            Serial.println("  s: start stop sequence");
            Serial.println("  h: home motor");
            Serial.println("  e: emergency stop");
            Serial.println("  t: autotune");
            Serial.println("  rp: read pressure");
            Serial.println("  gp: graph pressure");
            Serial.println("  om: operation mode");
            Serial.println("  start: start sequence");
            Serial.println("  stop: stop sequence");
            Serial.println("  help: display this help");   
        }
}



void handleCommandValues(VentCtl *ventilator,String cmd, String device, float value){
    if(cmd.equals("SET")){
        if(device.equals("CTL")){ ventilator->iface.cmode = (ControlMode)value; }
        else if(device.equals("OPR")){ ventilator->iface.omode = (OperationMode)value; }
        else if(device.equals("FR")){ ventilator->iface.pms.FR = value; }
        else if(device.equals("FiO2")){ ventilator->iface.pms.FIO2 = value; }
        else if(device.equals("PEEP")){ ventilator->iface.pms.PEEP = value; }
        else if(device.equals("TV")){ ventilator->iface.pms.TV = value; }
        else if(device.equals("IF")){ ventilator->iface.pms.IF = value; }
        else if(device.equals("TI")){ ventilator->iface.pms.TI = value; }
        else if(device.equals("PINS")){ ventilator->iface.pms.PINS = value; }
        else if(device.equals("R")){ ventilator->iface.pms.R = value; }
        EEPROM.put(0,ventilator->iface);
        Serial.println("ACK");
    }
    else if(cmd.equals("GET")){
        if(device.equals("CTL")){ Serial.println("CTL "+String(ventilator->iface.cmode)); }
        else if(device.equals("OPR")){ Serial.println("OPR "+String(ventilator->iface.omode)); }
        else if(device.equals("FR")){ Serial.println("FR "+String(ventilator->iface.pms.FR)); }
        else if(device.equals("FiO2")){ Serial.println("FiO2 "+String(ventilator->iface.pms.FIO2));}
        else if(device.equals("PEEP")){ Serial.println("PEEP "+String(ventilator->iface.pms.PEEP));  }
        else if(device.equals("TV")){ Serial.println("TV "+String(ventilator->iface.pms.TV)); }
        else if(device.equals("IF")){ Serial.println("IF "+String(ventilator->iface.pms.IF)); }
        else if(device.equals("TI")){ Serial.println("TI "+String(ventilator->iface.pms.TI)); }
        else if(device.equals("PINS")){ Serial.println("PINS "+String(ventilator->iface.pms.PINS));}
        else if(device.equals("R")){ Serial.println("R "+String(ventilator->iface.pms.R)); }
        else if(device.equals("STATE")){ ventilator->getState(ventilator->state); }
    }
}

void getSerialParams(VentCtl *ventilator,String inputString,bool stringComplete, bool hasCmdValue, short params){
   if(stringComplete){
        String cmd;
        String device;
        String value;
        if(hasCmdValue) {
                switch (params)
                {
                case 1:
                    cmd = inputString.substring(0,inputString.indexOf(" "));
                    inputString = inputString.substring(cmd.length()+1);
                    device = "NONE";
                    value = inputString.trim();
                    break;
                case 2:
                    cmd = inputString.substring(0,inputString.indexOf(" "));
                    inputString = inputString.substring(cmd.length()+1);
                    device = inputString.substring(0,inputString.indexOf(" "));
                    inputString = inputString.substring(device.length()+1);
                    value = inputString.trim();
                    break;              
                default:
                    break;
                }
            }
        else {
            cmd = inputString.trim();
            device = "NONE";
            value = "NONE";
        }
        inputString = "";
        stringComplete = false;
        hasCmdValue = false;
        Serial.println(String("command is: "+cmd+", argument is: "+device+" and value is: "+value));
        if(cmd.indexOf("SET") < 0 && cmd.indexOf("GET") < 0){
            int deviceInt = device.toInt();
            int valueInt = value.toInt();
            handleManualCommands(ventilator,cmd, deviceInt, valueInt);  
        }
        else{
            float valueFl = value.toFloat();
            handleCommandValues(ventilator,cmd,device,valueFl);
        } 
}

}

// steppers motion functions ------------------------------------------------------

// MOTION CONTROL FUNCTIONS FOR STEPPERS!

void initMotors(){
    motor_in.setMaxSpeed(500);
    motor_in.setAcceleration(5000);
    motor_out.setMaxSpeed(500);
    motor_out.setAcceleration(5000);
}

 void moveMotor(double value,int device){
      float steps = (value / 360.0)*PULSES_REV*-1;
    switch(device){
        case 1:
            if(!controller_in.isRunning()){
                motor_in.setTargetAbs(steps);
                controller_in.moveAsync(motor_in);
              //  Serial.println(String("Started motor_in in pos "+String(motor_in.getPosition())+" and moved to "+String(steps)+" steps"));
            } 
           // else Serial.println("motor_in is already moving!");
            break;
        case 2:
            if(!controller_out.isRunning()){
                motor_out.setTargetAbs(steps);
                controller_out.moveAsync(motor_out);
            }
            break;
        case 3:
            if(!controller_o2.isRunning()){
                motor_o2.setTargetAbs(steps);
                controller_o2.moveAsync(motor_o2);
            } 
            break;
        default:Serial.println("Invalid device!"); break;
    }
}

void setAllMotorHome(){
    motor_in.setPosition(0);
    motor_out.setPosition(0);
    motor_o2.setPosition(0);
}
void setMotorHome(int m){
    switch (m)
    {
    case 1:
        motor_in.setPosition(0);
        break;

    case 2:
        motor_out.setPosition(0);
        break;

    case 3:
        motor_o2.setPosition(0);
        break;
    
    default:
        break;
    }
}
int getPosition(int m){
    switch (m)
    {
    case 1:
        return motor_in.getPosition();
        break;

    case 2:
        return motor_out.getPosition();
        break;
    
    case 3:
        return motor_o2.getPosition();
        break;
    default:
        break;
    }
}

void setPosition(int m,int val){
    switch (m)
    {
    case 1:
        motor_in.setPosition(val);
        break;

    case 2:
        motor_out.setPosition(val);
        break;

    case 3:
        motor_out.setPosition(val);
        break;
    
    default:
        break;
    }
}
#include "processStates.h"

bool auxInitialized = false;
int countCycles = 0;
int graphPidCount = 0;
unsigned int pidTime = 0;
float filter = 0;

double PID1_Input, PID1_Output,PID1_Setpoint;
double PID2_Input, PID2_Output,PID2_Setpoint;
// Main PID function for closed loop control
PID mainPID(&PID1_Input,&PID1_Output,&PID1_Setpoint,0.0,0.0,0.0,DIRECT);
PID auxPID(&PID2_Input,&PID2_Output,&PID2_Setpoint,0.0,0.0,0.0,DIRECT);

void initPidTime(){
    pidTime = millis();
}
void initAux(){
    auxInitialized = false;
}
bool getAuxStatus(){
    return auxInitialized;
}
void graphPid(){
    if(graphPidCount >= 100){ pidTime = millis(); graphPidCount = 0;}
    unsigned long time = millis()-pidTime;
    Serial.print("#PI "); Serial.print(PID1_Input);
    Serial.print(" #PO1 "); Serial.print(PID1_Output);
    Serial.print(" #PO2 "); Serial.print(PID2_Output);
    Serial.print(" #T "); Serial.println(time);
    graphPidCount++;
}


void homeAll(VentCtl * ventilator, int pos){
    elapsedMillis homeTime = 0;
    moveMotor(pos,1);
    moveMotor(pos,2);
    moveMotor(pos,3);
    while(getPosition(1) != pos || getPosition(2) != pos){

    }
    setAllMotorHome();
    ventilator->setState(HOMED);
    //else{ Serial.println("Homing error! Time was overpassed!"); ventilator->setState(HOMED);}

}




void stop(VentCtl *ventilator){
    homeAll(ventilator,-90);
    ventilator->setState(STOPPED);
}
// PID TUNING FOR VENTILATOR
void selectCtl(VentCtl *ventilator,TestMode test){

ClosedCtl ctl;
switch (ventilator->iface.atune){
    case PRESSURE_IN:
        ctl = ventilator->pressure_in;
        break;
    case FLOW_IN:
        //Serial.println("Selecting for flow in");
        ctl = ventilator->flow_in;
        break;
    case PRESSURE_OUT:
        Serial.println("Selecting for pressure out");
        ctl = ventilator->pressure_out;
        break;
    case FLOW_OUT:
        Serial.println("Selecting for flow out");
        ctl = ventilator->flow_out;
        break;
    case O2:
        Serial.println("Selecting for O2");
        ctl = ventilator->o2;
        break;

    default:Serial.println("Something went wrong in selecting ctl mode!"); break;
}
switch (test){
    case TUNE: autoTunePID(ventilator,3,SAMPLE_TIME,ctl); break;
    case INIT: InitMainPID(ventilator,ctl); break;
    case EXEC: 
        HandleMainPID(ventilator,ctl);
        if(ventilator->iface.cmode == PRESSURE_CTL){
            if(PID1_Input > (PID1_Setpoint+2)){
                if(!auxInitialized){ 
                    ventilator->pressure_out.pid_params.setPoint = ventilator->pressure_in.pid_params.setPoint;
                    InitAuxPID(ventilator,ventilator->pressure_out);
                    }
                else HandleAuxPID(ventilator,ventilator->pressure_out);
            }
        }
     break;

    default:
        break;
    }

}

void autoTunePID(VentCtl *ventilator, float percentage, long loopInterval, ClosedCtl ctl){
    // Initialize min and max values of sensors according to control type
     PIDAutotuner tuner = PIDAutotuner();

    // Set the target value to tune to
    // This will depend on what you are tuning. This should be set to a value within
    // the usual range of the setpoint. For low-inertia systems, values at the lower
    // end of this range usually give better results. For anything else, start with a
    // value at the middle of the range.
    float inputValue = ctl.pid_params.setPoint / (1+(percentage/100.0));
    tuner.setTargetInputValue(inputValue);

    // Set the loop interval in microseconds
    // This must be the same as the interval the PID control loop will run at
    tuner.setLoopInterval(loopInterval);

    // Set the output range
    // These are the maximum and minimum possible output values of whatever you are
    // using to control the system (analogWrite is 0-255)
    tuner.setOutputRange(ctl.motor_params.minVal,ctl.motor_params.maxVal);
    Serial.println(String("Range is from "+String(ctl.motor_params.minVal)+" to "+String(ctl.motor_params.maxVal)));

    // Set the Ziegler-Nichols tuning mode
    // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
    // or PIDAutotuner::ZNModeNoOvershoot. Test with ZNModeBasicPID first, but if there
    // is too much overshoot you can try the others.
    tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);

    // This must be called immediately before the tuning loop
    tuner.startTuningLoop();

    // Run a loop until tuner.isFinished() returns true
    //long microseconds;
    long milliseconds;
    while (!tuner.isFinished()) {

        // This loop must run at the same speed as the PID control loop being tuned
        //long prevMicroseconds = microseconds;
        long prevMilliseconds = milliseconds;
       // microseconds = micros();
        milliseconds = millis();
        getValues(ventilator);
        //graph();

        // Get input value here (temperature, encoder position, velocity, etc)
        double input = fabs(selectSensorValue(ventilator,ctl.sensor));   
         Serial.print("Input is "+String(input));
        // Call tunePID() with the input value
        double output = tuner.tunePID(input);
        Serial.println(" Output is: "+String(output));
       // Serial.println(" Loop Time is: "+String(rate));

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        selectMotorMove(ctl.sensor,output,false);
        //controller->getCurrentSpeed(motor);
        //controller->getCurrentSpeed();

        // This loop must run at the same speed as the PID control loop being tuned
        while (millis() - milliseconds < loopInterval) delay(1);
    }

    // Turn the output off here.
     //controller->stopAsync(); 

    // Get PID gains - set your PID controller's gains to these
    ctl.pid_params.kp = tuner.getKp();
    ctl.pid_params.ki = tuner.getKi();
    ctl.pid_params.kd = tuner.getKd();
    Serial.println(String("Tuning parameters kp,ki and kd are: "+String(ctl.pid_params.kp)+" , "+String(ctl.pid_params.ki)+" , "+String(ctl.pid_params.kd)));
}


void InitMainPID(VentCtl *ventilator,ClosedCtl ctl){
      mainPID.SetTunings(ctl.pid_params.kp,ctl.pid_params.ki,ctl.pid_params.kd);
      mainPID.SetOutputLimits(ctl.motor_params.minVal,ctl.motor_params.maxVal);
      PID1_Input = selectSensorValue(ventilator,ctl.sensor);
      PID1_Setpoint = ctl.pid_params.setPoint;
      mainPID.SetSampleTime(SAMPLE_TIME); 
      mainPID.SetMode(AUTOMATIC);
      if(ventilator->state == INIT_EXHALING){mainPID.SetControllerDirection(REVERSE);}
      else{mainPID.SetControllerDirection(DIRECT);}
      countCycles = 0;         
}

void HandleMainPID(VentCtl *ventilator,ClosedCtl ctl){
    PID1_Input = selectSensorValue(ventilator,ctl.sensor);
    mainPID.Compute();
    if(PID1_Input < PID1_Setpoint){countCycles++;}
    // if(ventilator->state == EXHALING){
    // Serial.println("Input is: "+String(PID1_Input)+" Setpoint is "+String(PID1_Setpoint)+" and output is "+String(PID1_Output));
    // }

    selectMotorMove(ctl.sensor,PID1_Output,false);     
}

void checkForAux(VentCtl *ventilator){
    switch (ventilator->state)
    {
    case INHALING:
        handleInhaleAux(ventilator);
        break;

    case EXHALING:
        handleExhaleAux(ventilator);
        break;
    
    default:
        break;
    }

}

void handleInhaleAux(VentCtl *ventilator){
   // if(PID1_Input > (PID1_Setpoint)){
        if(!getAuxStatus()){ 
            ventilator->pressure_out.pid_params.setPoint = ventilator->pressure_in.pid_params.setPoint;
            InitAuxPID(ventilator,ventilator->pressure_out);
       }
        else HandleAuxPID(ventilator,ventilator->pressure_out);
  //  }
}

void handleExhaleAux(VentCtl *ventilator){
   // if(PID1_Input < (PID1_Setpoint-1)){
        if(!getAuxStatus()){ 
            ventilator->pressure_in.pid_params.setPoint = ventilator->pressure_out.pid_params.setPoint;
            InitAuxPID(ventilator,ventilator->pressure_in);
        }
        else HandleAuxPID(ventilator,ventilator->pressure_in);
  //  }
    //else selectMotorMove(ventilator->pressure_in.sensor,0);
}
void getCountCycles(VentCtl *ventilator){
    Serial.println("Cycles beofre setpoint = "+String(countCycles)); ventilator->setState(HOMING);
}
void InitAuxPID(VentCtl *ventilator, ClosedCtl ctl){
      auxPID.SetTunings(ctl.pid_params.kp,ctl.pid_params.ki,ctl.pid_params.kd);
      auxPID.SetOutputLimits(ctl.motor_params.minVal,ctl.motor_params.maxVal);
      PID2_Input = selectSensorValue(ventilator,ctl.sensor);
      PID2_Setpoint = ctl.pid_params.setPoint;
      auxPID.SetSampleTime(SAMPLE_TIME); 
      auxPID.SetMode(AUTOMATIC); 
      if(ventilator->state == INHALING){
      auxPID.SetControllerDirection(REVERSE);
      }
      else if(ventilator->state == EXHALING){
          auxPID.SetControllerDirection(DIRECT);
        }
      auxInitialized = true;
         
}

void HandleAuxPID(VentCtl *ventilator, ClosedCtl ctl){
    PID2_Input = selectSensorValue(ventilator,ctl.sensor);
    auxPID.Compute();
    //if(PID1_Input < PID1_Setpoint){countCycles++;}
    selectMotorMove(ctl.sensor,PID2_Output,false);

    
}

void getValues(VentCtl *ventilator){
    //ventilator->flow_in.sensor.currentValue = ventilator->getFlow(1);
    ventilator->flow_in.sensor.currentValue = 0;
    float pressure = ((analogRead(A0)/4095.0)*70.0)-6;
     filter = 0.9568*filter +0.04321*pressure;
    ventilator->pressure_in.sensor.currentValue = filter;
    ventilator->pressure_out.sensor.currentValue = ventilator->pressure_in.sensor.currentValue;
}


double selectSensorValue(VentCtl *ventilator,Sensor sensor){
    switch (sensor.sensorType)
    {
        case FLOW_IN: return ventilator->flow_in.sensor.currentValue;  break;
        case FLOW_OUT: return ventilator->flow_out.sensor.currentValue;  break;
        case PRESSURE_IN: return ventilator->pressure_in.sensor.currentValue;  break;
        case PRESSURE_OUT: return ventilator->pressure_out.sensor.currentValue; break;
        
        default:return ventilator->flow_in.sensor.currentValue; break;
        } 
}

void selectMotorMove(Sensor sensor, double output, bool home){
    int m = 0;
      switch (sensor.sensorType)
        {
        case PRESSURE_IN: m = 1; break;
        case PRESSURE_OUT: m = 2; break;
        case FLOW_IN:  m = 1; break;
        case FLOW_OUT: m = 2; break;
        case O2: m = 3; break;
        
        default:
            break;
        }
        moveMotor(output,m); 
}
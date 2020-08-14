#include <Arduino.h>
#include <Wire.h>
//#include "Plotter.h"
#include "VentCtl.h"
#include "interface.h"
#include "Graph.h"
#include "processStates.h"




// FLOW PARAMETER FOR PID: KP = 0.07, KI = 0.00 AND KD = 3.06
// FLOW OUTPUT PARAMETERS FOR PID KP = 20.08, KI = 1.18 KD = 85.37
//FOW OUTPUT IS 44.81 , 2.64 , 190.48


#define GRAPH_LEN 100
#define GRAPH_TIME 50

// #define SLAVE_ADDR 0x58
// #define OUTPUT_MIN 0x666        // 10%
// #define OUTPUT_MAX 0x399A       // 90% of 2^14 - 1
// #define PRESSURE_MIN 0      // min is 0 for sensors that give absolute values
// // #define PRESSURE_MAX 689476.0   // 30psi (and we want results in pascals)
// #define PRESSURE_MAX 100  // 30psi (and we want results in pascals)

uint32_t prev = 0; 
const uint32_t interval = 5000;

// Graph points
Point pressure,flow,o2[GRAPH_LEN];


// Alarm output pin
constexpr int buzzer = 23;

// Cylinder control
constexpr int cylinder_in = 22, cylinder_out = 21;

// Stop inputs
constexpr int eStop = 20;

// Start input
constexpr int start = 15;
// Analog input for O2 sensor
constexpr int o2_sensor = A0;


// stepper and controller's object declaration

// pin to stop the motor, connect a push button to this pin
constexpr int stopPin = 0;

// stopwatches 
elapsedMillis displayStopwatch = 0;  // timing the display of the current position
elapsedMillis blinkStopwatch = 0;    // timing the heartbeat LED
elapsedMillis debounceTimer = 0;     // debouncing input pins
elapsedMillis statesTimer = 0;
elapsedMillis graphTimer = 0;

// Plotted variables must be declared as globals 
double x;
double y;


IntervalTimer P_timer, F_timer;

int lastPos = 0;
bool hasCmdValue = false;
short params = 0;

// Buffers for saving graph data
float flowGraph[GRAPH_LEN];
float pressureGraph[GRAPH_LEN];
int flowBuffer, pressureBuffer;
uint8_t graphCount = 0;
unsigned long stTime = 0;


String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// Also declare plotter as global
//Plotter p;

VentCtl ventilator;

Omron_D6FPH flowSensor_in,flowSensor_out;

void InitMotorParams();
void handlePins();
void processStates();
void graph();
void graph_buffer();


// Counters for Respiratory frequency
uint16_t InsCount = 0;
uint16_t ExpCount = 0;
uint16_t TestCount = 0;
float InspTime = 0.0;
float ExpTime = 0.0;
uint16_t SampleCount = 0;
uint16_t SampleTimer = 0;


void setup()
{
    // Serial.begin();
     while (!Serial);
    Serial.println("Simple Serial Stepper Example");    
    Serial.println("(type help for help)");
    flowBuffer = 0;
    pressureBuffer = 0;
    pinMode(start,INPUT);
    pinMode(eStop,INPUT);
    initMotors();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(stopPin, INPUT_PULLUP);  // touch the pin with GND to stop the motor

    //p.Begin();
    // open I2C communication
    Wire.begin();
    ventilator.Init();
    flowSensor_in.begin();
    flowSensor_out.begin(Wire,0x0D,MODEL_0505AD3);
    ventilator.flowSensor_in = &flowSensor_in;
    ventilator.flowSensor_out = &flowSensor_out;
    InitMotorParams();
    EEPROM.get(0,ventilator.iface);
    
 
    // // Add time graphs. Notice the effect of points displayed on the time scale
     //p.AddTimeGraph( "Grafica de Sensor de presion (50,000 puntos)", 50000, "presion", x );
    // p.AddTimeGraph( "Time graph w/ 200 points", 200, "x label", x );
    graphTimer = 0;
    statesTimer = 0;
    ventilator.setState(IDLE);
    stTime = millis();
   initPidTime();
    SampleTimer = millis();
    analogReadResolution(12);

}


void loop()
{

    // handle input from pins ---------------------------------------------
    if(debounceTimer > 200){
        debounceTimer = 0;
       // handlePins();
    }

    // process state machine for ventilator ------------------------------
    if(statesTimer >= SAMPLE_TIME){
        statesTimer = 0;
        processStates();
        getValues(&ventilator);
        if(ventilator.state == INIT_PID_TEST || ventilator.state == PID_TESTING){ graphPid();}
        //graph();
    }

    // process graphics for ventilator ------------------------------------------------------------
    if(graphTimer >= GRAPH_TIME && (ventilator.state == INHALING || ventilator.state == EXHALING) ){
        graphTimer = 0;
        graphPid();
    }

    // the usual heartbeat ------------------------------------------------
    if (blinkStopwatch > 250)
    {
        blinkStopwatch = 0;
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN)); // toggle LED
    }

    // Update variables with arbitrary sine/cosine data
     unsigned long time = millis();
   // x = 10*sin( 2.0*PI*( time / 500.0 ));
   // x = (analogRead(A0)/65535.0)*37.0;



  

}

bool validateValues(){

return true;
}

void InitMotorParams(){
    ventilator.flow_out.motor_params.minVal = 0.0;
    ventilator.pressure_out.motor_params.minVal = -30.0;
    ventilator.flow_in.motor_params.minVal = 0.0;
    ventilator.pressure_in.motor_params.minVal = -30.0;
    ventilator.o2.motor_params.minVal = 0.0;
    ventilator.flow_out.motor_params.maxVal = 80.0;
    ventilator.pressure_out.motor_params.maxVal = 80.0;
    ventilator.flow_in.motor_params.maxVal = 50.0;
    ventilator.pressure_in.motor_params.maxVal = 50.0;
    ventilator.o2.motor_params.maxVal = 90.0*REDUCTION_O2;

    // MANUAL PARAMETERS FOR PID FLOW IN
    ventilator.flow_in.pid_params.kp = 0.04;
    ventilator.flow_in.pid_params.ki = 0.38;
    ventilator.flow_in.pid_params.kd = 0.0;
}

void pid_inhale_params(){
    ventilator.pressure_in.pid_params.kp = 1.42;
    ventilator.pressure_in.pid_params.ki = 0.0;
    ventilator.pressure_in.pid_params.kd =  0.02;
    ventilator.pressure_out.pid_params.kp = 4.8;
    ventilator.pressure_out.pid_params.ki = 0.0;
    ventilator.pressure_out.pid_params.kd =  0.0;
}

void pid_exhale_params(){
    ventilator.pressure_out.pid_params.kp = 7;
    ventilator.pressure_out.pid_params.ki = 0.0;
    ventilator.pressure_out.pid_params.kd =  0.0;
    ventilator.pressure_in.pid_params.kp = 1.42;
    ventilator.pressure_in.pid_params.ki = 0.0;
    ventilator.pressure_in.pid_params.kd =  0.0;
}


//------------------------------------------------
// Connect a pushbutton to the stop pin. 
// Stop command will be issued when pin is 
// pulled to GND.
// Only very simple debouncing implemented. Use "debounce2.h" or 
// similar for a real application

void handlePins()
{    if(ventilator.state == IDLE && digitalReadFast(start)){
        ventilator.setState(STARTING);
    }
    if(digitalReadFast(eStop)){
        ventilator.setState(STOPPING);
    }
}

// function for taking data into graph buffer ------------------------------------------
void graph_buffer(){
    if(flowBuffer < GRAPH_LEN){ flowGraph[flowBuffer] = analogRead(A0);flowBuffer++; }
    else{ Serial.write("#F"); Serial.write((uint8_t*)flowGraph, sizeof(flowGraph)); flowBuffer = 0;}
    if(pressureBuffer < GRAPH_LEN){ pressureGraph[pressureBuffer] = analogRead(A1); pressureBuffer++;}
    else{ Serial.write("#P"); Serial.write((uint8_t*)pressureGraph,sizeof(pressureGraph)); pressureBuffer = 0;}

}


void graph(){
    if(graphCount >= 100){ stTime = millis(); graphCount = 0;}
    unsigned long time = millis()-stTime;
    Serial.print("#F "); //Serial.print(ventilator.flow_in.sensor.currentValue,5);
    Serial.print(ventilator.flow_in.sensor.currentValue);
   // Serial.print(ventilator.getI2CInput(sensor.sensorType,sensor.minVal,sensor.maxVal));
   if(ventilator.pressure_in.sensor.currentValue < 0){ ventilator.pressure_in.sensor.currentValue = 0;}
    Serial.print(" #P "); Serial.print(ventilator.pressure_in.sensor.currentValue);
    Serial.print(" #T "); Serial.println(time);
    graphCount++;
}


// state machine processing function ------------------------------------------------------
void processStates(){
    switch (ventilator.state)
    {
    case HOMING: homeAll(&ventilator,-90); break;
    case HOMED: ventilator.setState(IDLE);  break;
    case STARTING: if(validateValues()){ ventilator.setState(STARTED);}
        else{ ventilator.setState(IDLE);}
     break;
    case STARTED:
        InspTime = ventilator.iface.pms.TI*1000.0;
        ExpTime =  (60.0/ventilator.iface.pms.FR)*1000 - InspTime;
        ventilator.setState(INIT_INHALING); break;
    case INIT_INHALING:
        initAux();
        InsCount = 0;
        pid_inhale_params(); 
        moveMotor(0,2);
        if(ventilator.iface.cmode == FLOW_CTL) InitMainPID(&ventilator,ventilator.flow_in);
        else if(ventilator.iface.cmode == PRESSURE_CTL){ 
            ventilator.pressure_in.pid_params.setPoint = ventilator.iface.pms.PINS;
            InitMainPID(&ventilator,ventilator.pressure_in);
            }
        ventilator.setState(INHALING);
      break;
    case INHALING:
        if(ventilator.iface.cmode == FLOW_CTL) HandleMainPID(&ventilator,ventilator.flow_in);
        else if(ventilator.iface.cmode == PRESSURE_CTL){
            HandleMainPID(&ventilator,ventilator.pressure_in);
            checkForAux(&ventilator);
            }
        if(++InsCount >= ((InspTime)/SAMPLE_TIME)){InsCount = 0; ventilator.setState(INIT_EXHALING);}
      break;
    case INIT_EXHALING:
        ExpCount = 0;
        initAux();
        pid_exhale_params();
        moveMotor(0,1); 
        if(ventilator.iface.cmode == FLOW_CTL) InitMainPID(&ventilator,ventilator.flow_out);
        else if(ventilator.iface.cmode == PRESSURE_CTL){
            ventilator.pressure_out.pid_params.setPoint = ventilator.iface.pms.PEEP;
             InitMainPID(&ventilator,ventilator.pressure_out);
        }
        ventilator.setState(EXHALING);
      break;
    case EXHALING:
        if(ventilator.iface.cmode == FLOW_CTL) HandleMainPID(&ventilator,ventilator.flow_out);
        else if(ventilator.iface.cmode == PRESSURE_CTL){
            HandleMainPID(&ventilator,ventilator.pressure_out);
            checkForAux(&ventilator);
        }
        if(++ExpCount >= ((ExpTime)/SAMPLE_TIME)){ExpCount = 0; ventilator.setState(INIT_INHALING);}
      break;
    case INIT_PID_TEST: initAux(); selectCtl(&ventilator,INIT); TestCount = 0; initPidTime(); ventilator.setState(PID_TESTING);  break;
    case PID_TESTING:
        if(++TestCount <= ventilator.iface.pms.testTime/SAMPLE_TIME){
            selectCtl(&ventilator,EXEC);  
        }
        else{getCountCycles(&ventilator);}
       // Serial.println("PID TIME IS"+String(millis()-pidTime));
      break;
    case TUNNING: selectCtl(&ventilator,TUNE); ventilator.setState(TUNED);  break;
    case TUNED: ventilator.setState(IDLE);  break;
    case STOPPING: stop(&ventilator); break;
    case STOPPED: ventilator.setState(IDLE);  break;
    case IDLE:  break;

    
    default:
        break;
    }
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if(inChar == ' '){
        hasCmdValue = true;
        params++;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
        stringComplete = true;
        getSerialParams(&ventilator,inputString,stringComplete,hasCmdValue,params);
        inputString = "";
        stringComplete = false;
        hasCmdValue = false;
        params = 0;  
    }
  }
}
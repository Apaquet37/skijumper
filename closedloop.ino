// Abby Paquette and Jesse Chan
// ENGS 147 final project

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include "ArduinoMotorShieldR3.h"
#include "AxisEncoderShield3.h"
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment


ArduinoMotorShieldR3 md;

#define refInput 0


#define DEV_I2C Wire
// #define LEDPIN 12
#define DESIRED_PWM 400
#define XSHUT_PIN 4 // Connect to XSHUT on the sensor

VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, XSHUT_PIN);
NAxisMotion mySensor;                 //Object that for the sensor

unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
//const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
const int streamPeriod = 10; // 100 Hz, fastest I can get euler I think, so every 10 ms
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream


enum State { // use a state machine 
    ON_RAMP,
    IN_AIR,
    LANDED,
    TIMEOUT
};

State currentState = ON_RAMP; // start on the ramp

unsigned long startTime = 0;



#define V_PWM_NPTS 19

#define SCALE 3.14*2.0/1440.0 // combined scale factor for converting to radians (2pi rad per 1440 encoder counts)
#define TIME_CONVERT 1.0e-6 // micros to seconds


#define N 20 // number of points in the PWM and voltage tables


// #define refInput 1.0472 // rad


#define D .05
#define F .025
#define TS 100000 // 100 ms in microseconds
#define RECORD_TIME 40000000 // 40 s total duration of loop, in microseconds
#define NUM_SAMPLES (RECORD_TIME / TS)
#define RAMP_EXIT_DELAY_US 500000 // leave ramp .5 s after reset

#define DEG_TO_RAD 0.0174533

#define GAIN 5.0 // K
#define B 1 // pole location
#define A .98 // zero location




void setup() {
  // put your setup code here, to run once:

  // Force Hardware Reset
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);   // Put sensor in shutdown mode
  delay(10);                      // Hold reset
  digitalWrite(XSHUT_PIN, HIGH);  // Wake sensor up
  delay(10);                      // Wait for boot

  // indicator LED
  // pinMode(LEDPIN, OUTPUT);

  // Initialize serial for output
  Serial.begin(115200);
  Serial.println("Starting...");

  // // Initialize I2C bus
  // DEV_I2C.begin();

  // // Configure VL53L4CD satellite component
  // sensor_vl53l4cd_sat.begin();

  // // Switch off VL53L4CD satellite component
  // sensor_vl53l4cd_sat.VL53L4CD_Off();

  // //Initialize VL53L4CD satellite component
  // Serial.println("Line before initializing sensor"); 
  // sensor_vl53l4cd_sat.InitSensor();
  // Serial.println("sensor ready"); // often gets stuck initializing ToF sensor

  // // running very fast
  // sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(10, 0);

  // // Start Measurements
  // sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

  md.init(); // initialize motor shield
  initEncoderShield(); // initialize encoder shield
  
  
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Sensor Initialization
  mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  
  delay(200); // ensure all config is complete
  Serial.println("setup done");
  Serial.println("Timestamp, angle, Error, Commanded_voltage, Commanded_pwm_value");

  

}

void loop() {
  // declare and initialize variables including storage array(s)
  // variables (keep declarations here unless you *REALLY* need a global...)
  static unsigned long progstart, prevloopstart;
  static bool running = false;                                 // allows us to enter TS if() block on first run through loop
  static unsigned long timeidx = 0;
  static uint16_t lastDistanceMm = 0;
  static bool hasValidDistance = false;
  
  static float times[NUM_SAMPLES];
  static long encoders[NUM_SAMPLES]; // store raw encoder values
  static float angle[NUM_SAMPLES]; // store converted radian values
  static float velocities[NUM_SAMPLES]; // rad/s
  static float velo_unconv[NUM_SAMPLES]; // pulse counts per microsecond
  static float vSignal[NUM_SAMPLES]; // commanded voltages
  static float pwmToSend[NUM_SAMPLES]; // commanded pwm values
  static float error[NUM_SAMPLES]; // errors
  
  unsigned long curtime, dt;  // note: long is a long INTEGER!
  long encoderValue = 0;
  float velocity, velocityUnConverted = 0;

  static unsigned long fullStart = micros();
  static unsigned long curTimeRamp = 0;

  uint8_t NewDataReady = 0;
  bool tofMeasurementValid = false;
  VL53L4CD_Result_t results;
  uint8_t status;

  float rollRad = 0;
  uint16_t distanceMm = lastDistanceMm;


  mySensor.updateGravAccel();    //Update the Gravity Acceleration data
  mySensor.updateCalibStatus();  //Update the Calibration Status


  //float theta = atan2(mySensor.readGravAccelX(), mySensor.readGravAccelZ()) * 180.0 / PI;
  float theta = atan2(mySensor.readGravAccelX(), mySensor.readGravAccelZ());


  switch(currentState) {

      case ON_RAMP:
        // motor off
        md.setM1Speed(0);
        md.setM2Speed(0);



        // switch condition is elapsed time since reset, independent of ToF reliability
        curTimeRamp = micros();
        if(micros()-fullStart >= RAMP_EXIT_DELAY_US) {
          currentState = IN_AIR;
        
          // debug
          startTime = micros(); // take note of when motor turns on
          progstart = startTime;
          prevloopstart = startTime;
          running = false; // reset for first control loop iteration
          timeidx = 0; // reset sample index for new run
        }

        break;

      case IN_AIR:
        // motor control loop - runs at TS frequency
        curtime = micros();

        // enforce loop timing
        if (!running || (curtime - prevloopstart) >= TS) { // OR conditions - start or 10ms

          // TIME CRITICAL OPERATIONS
          // * read encoder (position ish) long
          encoderValue = getEncoderValue(1);
          // store time, position, and raw encoder values
          times[timeidx] = ((float)curtime)*TIME_CONVERT; // in seconds
          encoders[timeidx] = -encoderValue; // negative here because of direction of encoder vs motor
          angle[timeidx] = (encoders[timeidx] - encoders[0])*SCALE; // turn into radians, make relative angle to start

          error[timeidx] = refInput - theta; // error in rad

          Serial.print(micros());
          Serial.print(",   ");   
          Serial.print(theta,   2);
          Serial.print(",   ");
          Serial.print(error[timeidx]);
          Serial.print(",   ");



          // compensator as difference equation
          // if(timeidx == 0){
          //   vSignal[timeidx] = GAIN*error[timeidx]; // ek-1 and vk-1 start at zero
          // } else {
          //   vSignal[timeidx] = B*vSignal[timeidx-1] + GAIN*error[timeidx] - A*GAIN*error[timeidx-1];
          // } 


          // just proportional control
          vSignal[timeidx] = GAIN*error[timeidx];
          
          pwmToSend[timeidx] = voltageToPWM(vSignal[timeidx]);
          
          Serial.print(vSignal[timeidx]);
          Serial.print(",   ");

          Serial.println(pwmToSend[timeidx]);



          md.setM1Speed((int)pwmToSend[timeidx]);
          md.setM2Speed((int)pwmToSend[timeidx]);

          // increment indices, pointers, etc.
          timeidx++;
          // and take care of other items that are not time critical
          running = true;
          prevloopstart = curtime;  // absolute count of when this sampling period started

          // check for sample time and array overruns if desired
          if(timeidx >= NUM_SAMPLES){
            // stop running
            Serial.println("Sample/array overrun");
            md.setM1Speed(0);
            md.setM2Speed(0);
            currentState = LANDED; // exit to landed state
          }
        }

        // Check for exit condition
        if(curtime >= (progstart + RECORD_TIME)) {
          currentState = LANDED;
          Serial.println("Recording time complete, landing");
          md.setM1Speed(0);
          md.setM2Speed(0);
        }
        break;

      case LANDED:

        // continue to have motor off
        // redundant command, but here just in case
        md.setM1Speed(0);
      
        break;

      // timeout and landed cases are currently the same, might collapse them into one, but for now preserving them as separate states
      case TIMEOUT:

        md.setM1Speed(0);

        break;
  }

}

// function receives voltage as an input and returns pwm as an output
// control loop can call this after computing voltage 
// U(s)/V(s)
int voltageToPWM(float V) {
  // hard-coded V, pwm values from calibration
  //float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-4.70,-3.70,-2.48,2.77,3.90,4.82,5.61,6.74,7.39,7.87,8.76};
  //float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-4.70,-3.70,-.5, -.25, .25, .5,3.90,4.82,5.61,6.74,7.39,7.87,8.76}; // good for zero
  //float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-.35,-.25,-.15,.15,.25,.35,5.61,6.74,7.39,7.87,8.76};
  float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-.5,-.4,-.3,.3,.4,.5,5.61,6.74,7.39,7.87,8.76}; // for 90?
  float pwm_arr[V_PWM_NPTS] = {-400,-350,-300,-275,-250,-200,-175,-150,-125,0, 0, 125,150,175,200,250,300,350,400};
  float slope;
  int i, res_pwm;

  // branch to find what linear-interp segment the passed-in V falls in
  for (i=0;i<V_PWM_NPTS-1;i++){
    // in loop, will run through comparing V to each next value; the last time it runs will be the last slope value saved
    // extra logical OR ensures voltages larger than v_max from v_arr follow the slope between the last two points
    if((V<v_arr[i+1]) || ((V > v_arr[i+1]) && (i==V_PWM_NPTS-2))){
      slope = (float)(pwm_arr[i+1]-pwm_arr[i])/(v_arr[i+1]-v_arr[i]);
      res_pwm = pwm_arr[i] + (int)(slope * (V-v_arr[i])+.5f);         // .5f to round to nearest integer
      break;
    }
  }
  // ensure nothing below -400 or above 400 is returned
  if(res_pwm < pwm_arr[0])                  res_pwm = pwm_arr[0];
  else if (res_pwm > pwm_arr[V_PWM_NPTS-1]) res_pwm = pwm_arr[V_PWM_NPTS-1];
  return(res_pwm);
}

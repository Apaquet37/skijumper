// Abby Paquette and Jesse Chan
// ENGS 147 final project

#include <Arduino.h>
#include <Wire.h>

#include "ArduinoMotorShieldR3.h"
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment


ArduinoMotorShieldR3 md;

#define refInput 0

NAxisMotion mySensor;                 //Object that for the sensor


enum State { // use a state machine 
  ON_RAMP,
  IN_AIR,
  LANDED
};

State currentState = ON_RAMP; // start on the ramp



#define V_PWM_NPTS 19

#define TS 100000 // 100 ms in microseconds
#define RECORD_TIME 40000000 // 40 s total duration of loop, in microseconds
#define NUM_SAMPLES (RECORD_TIME / TS)
#define RAMP_EXIT_DELAY_US 500000 // leave ramp .5 s after reset

#define GAIN 5.0 // K
#define B 1 // pole location
#define A .98 // zero location




void setup() {
  // Initialize serial for output
  Serial.begin(115200);
  Serial.println("Starting...");

  md.init(); // initialize motor shield
  
  
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Sensor Initialization
  mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  
  delay(200); // ensure all config is complete
  Serial.println("setup done");
  Serial.println("Timestamp, angle, Error, Commanded_voltage, Commanded_pwm_value"); // heading for data formatting

  

}

void loop() {
  // declare and initialize variables including storage array(s)
  // variables (keep declarations here unless you *REALLY* need a global...)
  static unsigned long progstart, prevloopstart;
  static bool running = false;                                 // allows us to enter TS if() block on first run through loop
  static unsigned long timeidx = 0;
  
  static float vSignal[NUM_SAMPLES]; // commanded voltages
  static float pwmToSend[NUM_SAMPLES]; // commanded pwm values
  static float error[NUM_SAMPLES]; // errors
  
  unsigned long curtime;

  static unsigned long fullStart = micros();


  mySensor.updateGravAccel();    //Update the Gravity Acceleration data
  mySensor.updateCalibStatus();  //Update the Calibration Status


  //float theta = atan2(mySensor.readGravAccelX(), mySensor.readGravAccelZ()) * 180.0 / PI; // for degrees
  float theta = atan2(mySensor.readGravAccelX(), mySensor.readGravAccelZ());


  switch(currentState) {

      case ON_RAMP:
        // motor off
        md.setM1Speed(0);
        md.setM2Speed(0);



        // switch condition is elapsed time since reset, independent of ToF reliability
        if(micros()-fullStart >= RAMP_EXIT_DELAY_US) {
          currentState = IN_AIR;
        
          unsigned long now = micros();
          progstart = now;
          prevloopstart = now;
          running = false; // reset for first control loop iteration
          timeidx = 0; // reset sample index for new run
        }

        break;

      case IN_AIR:
        // motor control loop - runs at TS frequency
        curtime = micros();

        // enforce loop timing
        if (!running || (curtime - prevloopstart) >= TS) { // OR conditions - start or 10ms

          error[timeidx] = refInput - theta; // error in rad

          Serial.print(micros());
          Serial.print(",   ");   
          Serial.print(theta,   2);
          Serial.print(",   ");
          Serial.print(error[timeidx]);
          Serial.print(",   ");



          // compensator as difference equation PI
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

  }

}

// function receives voltage as an input and returns pwm as an output
// control loop can call this after computing voltage 
// U(s)/V(s)
int voltageToPWM(float V) {
  // hard-coded V, pwm values from calibration
  //float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-4.70,-3.70,-2.48,2.77,3.90,4.82,5.61,6.74,7.39,7.87,8.76}; // the original array
  //float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-4.70,-3.70,-.5, -.25, .25, .5,3.90,4.82,5.61,6.74,7.39,7.87,8.76}; // varying deadbands and zero regions
  float v_arr[V_PWM_NPTS] =  {-8.73,-7.86,-7.33,-6.98,-6.65,-5.57,-.5,-.4,-.3,.3,.4,.5,5.61,6.74,7.39,7.87,8.76}; // varying deadbands and zero regions
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

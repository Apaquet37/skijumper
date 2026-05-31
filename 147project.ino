// Abby Paquette
// ENGS 147 Ski Jumper Project


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

#define DEV_I2C Wire
// #define LEDPIN 12
#define SHUT_OFF_TIME 500000 // .5 seconds
#define DESIRED_PWM 450
#define XSHUT_PIN 4 // Connect to XSHUT on the sensor

ArduinoMotorShieldR3 md;
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


void setup() {
  // put your setup code here, to run once:

  // 1. Force Hardware Reset
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

  // Initialize I2C bus
  DEV_I2C.begin();

  // Configure VL53L4CD satellite component
  sensor_vl53l4cd_sat.begin();

  // Switch off VL53L4CD satellite component
  sensor_vl53l4cd_sat.VL53L4CD_Off();

  //Initialize VL53L4CD satellite component
  Serial.println("Line before initializing sensor"); 
  sensor_vl53l4cd_sat.InitSensor();
  Serial.println("sensor ready"); // often gets stuck initializing ToF sensor

  // running very fast
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(10, 0);

  // Start Measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

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

  md.setM1Speed(0); // make sure motor starts off
  // digitalWrite(LEDPIN, LOW); // led starts off

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t NewDataReady = 0;
  uint8_t validResults = 0;
  VL53L4CD_Result_t results;
  uint8_t status;


  // status 1 means no new data, 0 means there is new data
  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  // if there is new data 
  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);

    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    mySensor.updateEuler();

    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print("ms ");

    Serial.print(" heading(yaw): ");
    Serial.print(mySensor.readEulerHeading()); // Heading of the euler data

    Serial.print(" roll: ");
    Serial.print(mySensor.readEulerRoll());  // Roll of the euler data

    Serial.print(" pitch: ");
    Serial.print(mySensor.readEulerPitch());  // Pitch of the euler data

    Serial.print("      C: ");
    Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

    Serial.println();

    // flag for valid data
    if(results.range_status == 0){
      validResults = 1;
    }
    else{
      validResults = 0;
    }
  }

  if(validResults){ // only enter the state machine if there are existing and valid results
    switch(currentState) {

      case ON_RAMP:
        // motor off
        Serial.println("On ramp"); // debug
        md.setM1Speed(0);
        // digitalWrite(LEDPIN, LOW); // indicator led off


        // switch condition is that it leaves the ramp
        // could add in some debounce logic here to require two readings over 50
        if(results.distance_mm > 50) {
          currentState = IN_AIR;
        
          // debug
          Serial.println("Was on ramp now is in air");

          startTime = micros(); // take note of when motor turns on

          // start motor
          md.setM1Speed(DESIRED_PWM);
          md.setM2Speed(DESIRED_PWM);
          // digitalWrite(LEDPIN, HIGH); // indicator led on
        }

        break;

      case IN_AIR:
        // motor on - don't keep resending pwm command right now
        // control will go here

        //Serial.println("In air");  // debug
        // digitalWrite(LEDPIN, HIGH); // indicator led on
        // digitalWrite(LEDPIN, LOW); // indicator led on
        md.setM1Speed(DESIRED_PWM);
        md.setM2Speed(DESIRED_PWM);

        
        

        // switch condition
        // if(results.distance_mm < 100) {
        //   currentState = LANDED;

        //   Serial.println("Was in air now landed");
          
        //   // turn off motor
        //   md.setM1Speed(0);
        //   digitalWrite(LEDPIN, LOW); // indicator LED off
        // }
        // // also include a safety shutoff
        // else if(micros() - startTime >= SHUT_OFF_TIME) {

        //   currentState = TIMEOUT;

        //   md.setM1Speed(0);
        // }

        break;

      case LANDED:

        // continue to have motor off
        // redundant command, but here just in case
        md.setM1Speed(0);
      
        Serial.println("Landed and motor off");

        /*
        // don't want this in this iteration, think if there should be a way to return to beginning
        // condition shouldn't be distance based though
        // think just pressing reset would be better
        if(results.distance_mm < 50) {
          currentState = ON_RAMP;
        }
        */
        break;

      // timeout and landed cases are currently the same, might collapse them into one, but for now preserving them as separate states
      case TIMEOUT:

        md.setM1Speed(0);

        /*
        if(results.distance_mm < 50) {
            currentState = ON_RAMP;
        }
        */
        break;
    }
  }

}

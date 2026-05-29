#include "NAxisMotion.h"
#include "Wire.h"
#include "ArduinoMotorShieldR3.h"
#include "AxisEncoderShield3.h"

#define DURATION 10000000
#define TSAMPLE1 10000  // 1: 10,000 us = 10 ms = .01 s - for the compensator designed by discrete equivalent
#define TSAMPLE2 100000 // 2: 400,000 us = 400 ms = .40 s - for the compensator designed by direct digital
#define NSAMPLES (DURATION / TSAMPLE2)
#define V_PWM_NPTS 18
#define VMIN -9.65
#define VMAX 9.33
#define R 100         // reference speed updated from lab 2 to now, 100 rad/s
#define A 0.07245     // two constants per difference equation to clean them up
#define B 0.06755
#define C 0.096
#define D 0.048

ArduinoMotorShieldR3 md;

void setup() {
  // put your setup code here, to run once:

  // peripheral initialization (initializing Serial connection)
  Serial.begin(115200);
  I2C.begin();

  // initialize the motor shield
  md.init();

  // initialize the encoder shield - not super necessary to do it to read one channel in particular bc you can just call for the one channel to be read...?
  initEncoderShield();

  // delay
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:

  // declare and initialize variables, including storage arrays

  // arrays
  long times[NSAMPLES];    
  double position[NSAMPLES];  
  double speed[NSAMPLES];
  double eofs[NSAMPLES];
  float vofs[NSAMPLES];
  int uofs[NSAMPLES];  

  // non-array(s)
  unsigned long progstart, prevloopstart, curtime, time_idx, i;
  int rofs;
  double encoder1value, deltapos, dt;
  float omegaofs;

  // set reference step input
  rofs = R;

  // set motor pwm to 0 + delay
  md.setM1Speed(0);
  delay(1000);

  // set timing variables going into the while loop
  curtime = micros();
  progstart = curtime;
  prevloopstart = curtime;
  time_idx = 0;


  // // WHILE LOOP FOR DESIGN BY DISCRETE EQUIVALENT (comment out when using direct digital)
  // while((curtime < progstart + DURATION) && (time_idx < NSAMPLES)) {
  //   // update time
  //   curtime = micros();

    
  //   if((curtime - prevloopstart) >= TSAMPLE1) {
      
  //     // get encoder value to compute position
  //     encoder1value = getEncoderValue(1);
  //     times[time_idx] = curtime;
  //     position[time_idx] = ((double)encoder1value / 1440.0) * (2.0 * PI); // this executes Theta(s)/X(s)

  //     // calculate speed
  //     if(time_idx == 0) {
  //       speed[time_idx] = 0.0;
  //       omegaofs = speed[time_idx];
  //       eofs[time_idx] = rofs - omegaofs;
  //       vofs[time_idx] = A*eofs[time_idx]; // is the right way to handle the first time through the ctrl loop to remove any past values from the equation?
  //     } 
  //     else {
  //       deltapos = position[time_idx] - position[(time_idx-1)];
  //       dt = ((double)times[time_idx] - (double)times[(time_idx-1)])/1000000.0;
  //       speed[time_idx] = -(double)deltapos / (double)dt;
  //       omegaofs = speed[time_idx];
  //       eofs[time_idx] = rofs - omegaofs;
  //       vofs[time_idx] = A*eofs[time_idx] - B*eofs[time_idx-1] + vofs[time_idx-1];
  //     }

  //     // anti-windup - clamp V(s) to what the battery can provide
  //     if (vofs[time_idx] > VMAX) vofs[time_idx] = VMAX;
  //     if (vofs[time_idx] < VMIN) vofs[time_idx] = VMIN;
    
  //     // calculate U(s), using the function
  //     uofs[time_idx] = vtopwm(vofs[time_idx]);

  //     // set motor speed to U(s)
  //     md.setM1Speed(uofs[time_idx]);

  //     // update timing and index
  //     prevloopstart = curtime;
  //     time_idx++;
  //   }
    
  //   // update time
  //   curtime = micros();
  // }

  // WHILE LOOP FOR DESIGN BY DIRECT DIGITAL (comment out when using discrete equivalent)
  while((curtime < progstart + DURATION) && (time_idx < NSAMPLES)) {
    // update time
    curtime = micros();

    if((curtime - prevloopstart) >= TSAMPLE2) {
      
      // get encoder value to compute position
      encoder1value = getEncoderValue(1);
      times[time_idx] = curtime;
      position[time_idx] = ((double)encoder1value / 1440.0) * (2.0 * PI); // this executes Theta(s)/X(s)

      // calculate speed
      if(time_idx == 0) {
        speed[time_idx] = 0.0;
        omegaofs = speed[time_idx];
        eofs[time_idx] = rofs - omegaofs;
        vofs[time_idx] = C*eofs[time_idx];  // is this the right way to handle the first pass...?
      } 
      else {
        deltapos = position[time_idx] - position[(time_idx-1)];
        dt = ((double)times[time_idx] - (double)times[(time_idx-1)])/1000000.0;
        speed[time_idx] = -(double)deltapos / (double)dt;
        omegaofs = speed[time_idx];
        eofs[time_idx] = rofs - omegaofs;
        vofs[time_idx] = C*eofs[time_idx] - D*eofs[time_idx-1] + vofs[time_idx-1];
      }

      if (vofs[time_idx] > VMAX) vofs[time_idx] = VMAX;
      if (vofs[time_idx] < VMIN) vofs[time_idx] = VMIN;

      // store speed in Omega(s) (MOVED INSIDE IF STATEMENT)
      // omegaofs = speed[time_idx];

      // calculate E(s) (MOVED INSIDE IF STATEMENT)
      // eofs[time_idx] = rofs - omegaofs;

      // calculate V(s) using difference equation for the compensator's z-domain tf (MOVED INSIDE IF STATEMENT)
      // vofs[time_idx] = A*eofs[time_idx] - A*B*eofs[time_idx-1] + vofs[time_idx-1];
    
      // calculate U(s), using the function
      uofs[time_idx] = vtopwm(vofs[time_idx]);

      // set motor speed to U(s)
      md.setM1Speed(uofs[time_idx]);

      // update timing and index
      prevloopstart = curtime;
      time_idx++;
    }
    
    // update time
    curtime = micros();
  }

  // stop motor after data is collected
  md.setM1Speed(0);

  // print data in CSV format
  Serial.println();
  Serial.println("time_s,speed_rad_s,error_rad_s,ctrl_effort_V,pwm");
  for(i=0;i<time_idx;i++){
    Serial.print((double)times[i] / 1000000.0, 6);  // s
    Serial.print(",");
    Serial.print(speed[i],20);
    Serial.print(",");
    Serial.print(eofs[i], 6);
    Serial.print(",");
    Serial.print(vofs[i], 6);
    Serial.print(",");
    Serial.println(uofs[i]);
  }

  // forever loop to prevent re-running
  while(true){
  }
}

int vtopwm(float V) {
  // hard-code (:() in V, pwm values from lab 1
  float v_arr[V_PWM_NPTS] =  {-9.65,-9.48,-9.25,-9.0,-8.65,-8.17,-7.33,-6.45,-4.7,4.32,6.06,7.05,8.02,8.44,8.75,9.03,9.18,9.33};
  float pwm_arr[V_PWM_NPTS] = {-400,-350,-300,-250,-200,-150,-100,-75,-50,50,75,100,150,200,250,300,350,400};
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

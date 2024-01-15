#include "Arduino.h"
#include "PID.h"

PID::PID(float Kp, float Ki, float Kd){
  KP = Kp;
  KI = Ki;
  KD = Kd;
}
  
float PID::calculate(int feedback){
  // Pre-calculation
  error = target - float(feedback);
  
  
  // Calculation
  switch(mode){
    case 0: // step
      error_sum += error*step_count;
      output = (KP * error) + (KI * error_sum) + (KD * (error-last_error)/step_count);
      step_count++; // Post-calculation
      break;
    case 1: // time
      error_sum += error*(millis()-time_flag);
      output = (KP * error) + (KI * error_sum) + (KD * (error-last_error)/(millis()-time_flag));
      break;
    default: // time
      error_sum += error*(millis()-time_flag);
      output = (KP * error) + (KI * error_sum) + (KD * (error-last_error)/(millis()-time_flag));
      break;
  }

  // Post-calculation
  last_error = error;
  return output;
}

void PID::set_target(float t){
  target = t;

  //set all the counter back to start when the target changed
  step_count = 1;
  time_flag = millis();
}

void PID::set_mode(int m){
  mode = m;
  
  //set all the counter back to start when the mode changed
  step_count = 1;
  time_flag = millis();
}

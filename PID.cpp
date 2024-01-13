#include "Arduino.h"
#include "PID.h"

PID::PID(float Kp, float Ki, float Kd){
  KP = Kp;
  KI = Ki;
  KD = Kd;
}
  
float PID::calculate(float feedback){
  // Pre-calculation
  error = target - feedback;
  //error_sum += error;
  
  // Calculation
  switch(mode){
    case 0: // step
      output = (KP * error) + (KI *  error*step_count) + (KD * (error-last_error)/step_count);
      step_count++; // Post-calculation
      break;
    case 1: // time
      output = (KP * error) + (KI *  error*(millis()-time_count)) + (KD * (error-last_error)/(millis()-time_count));
      break;
    default:
      output = (KP * error) + (KI *  error*step_count) + (KD * (error-last_error)/step_count);
      step_count++; // Post-calculation
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
  time_count = millis();
}

void PID::set_mode(int m){
  mode = m;
  
  //set all the counter back to start when the mode changed
  step_count = 1;
  time_count = millis();
}

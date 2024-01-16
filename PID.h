#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID{
  public:
    //constructor
    PID(float Kp, float Ki, float Kd);

    void set_target(float t);
    void set_mode(int m); //0: step, 1: time
    void set_max(int themax); //+max and -min
    
    float calculate(int feedback); // default mode: time

  private:
    //somehow constant
    float KP = 1, KI = 1, KD = 1;
    int mode = 0; //0: step, 1: time
    int max_point = 10000;
    float target = 0;

    //counter
    float step_count = 1;
    unsigned long time_flag = 0;

    //output things
    float output = 0;
    
    float error = 0;
    float last_error = 0;
    float error_sum = 0;
};

#endif //PID_H

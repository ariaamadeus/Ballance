#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID{
  public:
    //constructor
    PID(float Kp, float Ki, float Kd);

    void set_target(float t);
    void set_mode(int m);
    
    float calculate(float input);

  private:
    //somehow constant
    float KP = 1, KI = 1, KD = 1;
    int mode = 0; //0: step, 1: time
    float target = 0;

    //counter
    float step_count = 1;
    unsigned long time_count = 0;

    //output things
    float output = 0;
    
    float error = 0;
    float last_error = 0;
};

#endif //PID_H

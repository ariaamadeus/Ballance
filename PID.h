// MIT License

// Copyright (c) 2024 Universitas Kristen Krida Wacana
// Copyright (c) 2024 Aria Amadeus Salim <amadeus.aria@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

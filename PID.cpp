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

#include "Arduino.h"
#include "PID.h"

PID::PID(float Kp, float Ki, float Kd) {
  KP = Kp;
  KI = Ki;
  KD = Kd;
}

float PID::calculate(int feedback) {
  // Pre-calculation
  error = target - float(feedback);


  // Calculation
  switch (mode) {
    case 0: // step
      error_sum += error * step_count;
      output = (KP * error) + (KI * error_sum) + (KD * (error - last_error) / step_count);
      step_count++; // Post-calculation
      break;
    case 1: // time
      error_sum += error * (millis() - time_flag);
      //      output = (KP * error) + (KI * error_sum) + (KD * (error-last_error)/(millis()-time_flag));
      output = (KP * error) + (KI * error_sum) + (KD * (error - last_error));
      break;
    default: // time
      error_sum += error * (millis() - time_flag);
      //      output = (KP * error) + (KI * error_sum) + (KD * (error-last_error)/(millis()-time_flag));
      output = (KP * error) + (KI * error_sum) + (KD * (error - last_error));
      break;
  }

  // Post-calculation
  last_error = error;
  if (output > max_point) output = float(max_point);
  else if (output < (-1)*max_point) output = (-1) * float(max_point);
  return output;
}

void PID::set_target(float t) {
  target = t;

  //set all the counter back to start when the target changed
  step_count = 1;
  time_flag = millis();
}

void PID::set_mode(int m) {
  mode = m;

  //set all the counter back to start when the mode changed
  step_count = 1;
  time_flag = millis();
}

void PID::set_max(int themax) {
  max_point = themax;
}

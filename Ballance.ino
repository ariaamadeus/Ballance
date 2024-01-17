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

#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "PID.h"

#define stepPin 6
#define dirPin 5
#define enaPin 4
//#define echoPin A4 //SDA
//#define trigPin A5 //SCL

//Classes
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // use functions to step
Adafruit_VL53L0X ToF; //Time of Flight, distance sensor
PID pid(0.4, 0, 3); //Kp,Ki,Kd

//Variables
int avg[4] = {0, 0, 0, 0};

float dist;
long duration;

//Functions
//float ultrasonic_read() {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  duration = pulseIn(echoPin, HIGH);
//  return duration * 0.034 / 2;
//}

void setup() {
  Serial.begin(9600);

  while (!ToF.begin()) {
    Serial.println(F("Retrying to boot VL53L0X..."));
    delay(1000);
  }
  ToF.startRangeContinuous();

  //  pinMode(trigPin, OUTPUT);
  //  pinMode(echoPin, INPUT);

  stepper.setMaxSpeed(6400);
  stepper.setAcceleration(6400);
  stepper.setCurrentPosition(-120);
  pid.set_mode(1);//0: step, 1: time
  pid.set_target(180);
  pid.set_max(120); //max stepper
}

void loop() {
  dist = ToF.readRange();
  for (int x = 0; x < 3; x++) {
    avg[x] = avg[x + 1];
  }
  if (dist < 450) {
    avg[3] = dist;
  }
  else {
    avg[3] = 400;
  }
  dist = 0;
  for (int x = 0; x < 4; x++) {
    dist += avg[x];
  }
  dist /= 4;
  stepper.runToNewPosition(pid.calculate(dist));
  Serial.println(dist);
}

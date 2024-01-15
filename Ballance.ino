#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "PID.h"

//Stepper
#define stepPin 6
#define dirPin 5
#define enaPin 4

//Classes
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // use functions to step
Adafruit_VL53L0X ToF = Adafruit_VL53L0X(); //Time of Flight, distance sensor
PID pid(0.5, 0.0001, 0); //Kp,Ki,Kd

void setup() {
  Serial.begin(9600);

  while (!ToF.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  ToF.startRangeContinuous();

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(3200);

  pid.set_mode(1);//0: step, 1: time
  pid.set_target(120);
}

void loop() {
  //  stepper.runToNewPosition(pid.calculate(ToF.readRange()));
  int dist = ToF.readRange();
  if (dist < 500) {
    float thespeed = pid.calculate(dist);
//    stepper.setSpeed(thespeed);
    stepper.runToNewPosition(thespeed);
    Serial.println(thespeed);
//    stepper.runSpeed();
  }
}

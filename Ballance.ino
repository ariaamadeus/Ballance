#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "PID.h"

#define stepPin 6
#define dirPin 5
#define enaPin 4

long duration; // variable for the duration of sound wave travel

PID pid(1,1,1); //Kp,Ki,Kd

//Classes
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // use functions to step
Adafruit_VL53L0X ToF = Adafruit_VL53L0X(); //Time of Flight, distance sensor

float PID2Stepper(float input){
  float scale = 1.2; //ratio
  return input * scale;
}

void setup() {
  Serial.begin(9600);
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(3200);
  while (!ToF.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  ToF.startRangeContinuous();
  pid.set_target(120);
}

void loop() {
//  stepper.runToNewPosition(PID2Stepper(pid.calculate(ToF.readRange())));
  stepper.setSpeed(PID2Stepper(pid.calculate(ToF.readRange())));
  stepper.runSpeed();
}

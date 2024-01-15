#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include "PID.h"

//Stepper
#define stepPin 6
#define dirPin 5
#define enaPin 4

//Classes
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // use functions to step
Adafruit_VL53L0X ToF; //Time of Flight, distance sensor
PID pid(1,1,1); //Kp,Ki,Kd

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
  stepper.setSpeed(pid.calculate(ToF.readRange()));
  stepper.runSpeed();
}

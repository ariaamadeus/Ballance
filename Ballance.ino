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
PID pid(2, 0.01, 2); //Kp,Ki,Kd

//Variables
int dist;
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

  pid.set_mode(1);//0: step, 1: time
  pid.set_target(230);
  pid.set_max(500); //max stepper
}

void loop() {
  dist = ToF.readRange();
  if (dist < 500) stepper.runToNewPosition(pid.calculate(dist));
}

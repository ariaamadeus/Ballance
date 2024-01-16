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

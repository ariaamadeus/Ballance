#include <AccelStepper.h>
#include "PID.h"

//Ultrasonic
#define echoPin 18 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 5 //attach pin D3 Arduino to pin Trig of HC-SR04x

long duration; // variable for the duration of sound wave travel

PID pid(1,1,1); //Kp,Ki,Kd

AccelStepper stepper; // use functions to step

float PID2Stepper(float input){
  float scale = 1; //ratio
  return input * scale;
}

float ultrasonic_read(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  return duration * 0.034 / 2;
}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepper.setMaxSpeed(50);
  stepper.setAcceleration(50);
  pid.set_target(0);
}

void loop() {
  //stepper move to:
  stepper.moveTo(PID2Stepper(pid.calculate(ultrasonic_read())));
//  stepper.runSpeedToPosition();//?
//  stepper.run();//?
}

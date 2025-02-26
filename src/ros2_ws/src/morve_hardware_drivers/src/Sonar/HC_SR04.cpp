//
// Created by radek on 17.11.2024.
// inspired (used from): http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp
//

#include <wiringPi.h>
#include "morve_hardware_drivers/Sonar/HC_SR04.h"

int HC_SR04::count = 0;
volatile long long HC_SR04::startTimeUsec = 0;
volatile long long HC_SR04::travelTimeUsec = 0;
int HC_SR04::echo = 0;


void HC_SR04::EchoInterrupt(){
  long long time = micros();
  if(digitalRead(echo) == HIGH){
    startTimeUsec = time;
    travelTimeUsec = 0;
  } else{
    travelTimeUsec = time - startTimeUsec;
    startTimeUsec = 0;
  }
}

HC_SR04::HC_SR04(int trigger_pin, int echo_pin){
  trigger = trigger_pin;
  echo = echo_pin;
  // Set HC_SR04 id
  //id = echo;
  //count++;
  
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  wiringPiISR(echo, INT_EDGE_BOTH, HC_SR04::EchoInterrupt);
  digitalWrite(trigger, LOW);
  delay(500);
}

HC_SR04::~HC_SR04(){
  pinMode(echo, INPUT);
  pinMode(trigger, INPUT);
  //count--;
}

double HC_SR04::Distance(int timeout)
{
  delay(10);
  SendTrigger();

  delay(20); // 20 ms so the ultrasound has time to travel back

  //distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;
  distance_cm = travelTimeUsec * 340.29 / 20000;

  return distance_cm;
} 

long long HC_SR04::MeassureSensorTimeout(int timeout_ms){
  delay(10);
  SendTrigger();
  delay(timeout_ms);

  return travelTimeUsec;
}

void HC_SR04::SendTrigger()
{
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
}



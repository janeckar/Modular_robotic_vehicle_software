//
// Created by radek on 17.11.2024.
// inspired (used from): http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp
//

#include <string>
#include <wiringPi.h>
#include "morve_hardware_drivers/Sonar/HC_SR04.h"
#include "morve_hardware_drivers/hardware_error.h"

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

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  int res = wiringPiISR(echo, INT_EDGE_BOTH, HC_SR04::EchoInterrupt);
  if(res < 0){
    pinMode(trigger, INPUT);
    std::string msg = "Tried to register interrupt for GPIO pin number " + std::to_string(echo) + " but probably the GPIO pin is already in use.";
    throw hardware_error(msg);
  }
  digitalWrite(trigger, LOW);
  delay(500);
}

HC_SR04::~HC_SR04(){
  pinMode(echo, INPUT);
  pinMode(trigger, INPUT);
}

double HC_SR04::Distance(int measurement_time_ms)
{
  SendTrigger();

  if(measurement_time_ms < 30){ // 30 ms
    delay(30);
  } else{
    delay(measurement_time_ms);
  }

  return read_distance();
} 

long long HC_SR04::MeassureSensorTimeout(int measurement_time_ms){
  SendTrigger();

  if(measurement_time_ms < 30){ // 30 ms
    delay(30);
  } else{
    delay(measurement_time_ms);
  }

  return travelTimeUsec;
}

void HC_SR04::SendTrigger()
{
  digitalWrite(trigger, HIGH);
  delayMicroseconds(12); // 10 microseconds
  digitalWrite(trigger, LOW);
}

void HC_SR04::request_distance()
{
  SendTrigger();
}

double HC_SR04::read_distance()
{
  // distanceMeters = 100*((travelTimeUsec/1'000'000.0)*340.29)/2;
  distance_m = travelTimeUsec * 340.29 / 2'000'000;

  return distance_m;
} 

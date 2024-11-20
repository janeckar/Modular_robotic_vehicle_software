//
// Created by radek on 17.11.2024.
// inspired (used from): http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp
//

#include <iostream>
#include <wiringPi.h>
#include "HC_SR04.h"

int HC_SR04::count = 0;
volatile long long HC_SR04::startTime = 0;

void HC_SR04::EchoInterrupt(){
  long long time = micros();
  if(digitalRead(echo) == HIGH){
    startTime = time;
  } else{
    travelTimeUsec = time - startTime;
    startTime = 0;
  }
}

HC_SR04::HC_SR04(int trigger_pin, int echo_pin){
  trigger = trigger_pin;
  echo = echo_pin;
  // Set HC_SR04 id
  id = echo;
  count++;
  //startTime.push_back(0);
  
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  wiringPiISR(echo, INT_EDGE_BOTH, HC_SR04::EchoInterrupt);
  digitalWrite(trigger, LOW);
  delay(500);
}

HC_SR04::~HC_SR04(){
  count--;
}

double HC_SR04::Distance(int timeout)
{
  delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    delay(30); // 30 ms

    //now=micros();

    //while (digitalRead(echo) == LOW && micros()-now<timeout); // experimentally i found out that the propagation time on echo pin is less than 450 us, so safe bet is 1 ms
    //recordPulseLength();
    //travelTimeUsec = endTimeUsec - startTimeUsec;
    
    //int traveltime = endTimeUsec - now;
    //std::cout << "travel time" << travelTimeUsec << " " << traveltime << std::endl;
    //distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;
    distance_cm = travelTimeUsec * 340.29 / 20000;

    return distance_cm;
} 

void HC_SR04::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH);
    endTimeUsec = micros();
}



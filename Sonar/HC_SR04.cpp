//
// Created by radek on 17.11.2024.
// inspired (used from): http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp
//

#include <iostream>
#include <wiringPi.h>
#include "HC_SR04.h"

int HC_SR04::Travel_time_ms(){
  return 0;
}

HC_SR04::HC_SR04(int trigger_pin, int echo_pin){
  trigger = trigger_pin;
  echo = echo_pin;
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trigger, LOW);
  delay(500);
}


double HC_SR04::Distance(int timeout)
{
  delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    now=micros();

    while (digitalRead(echo) == LOW && micros()-now<timeout);
    
    recordPulseLength();

    travelTimeUsec = endTimeUsec - startTimeUsec;
    //int traveltime = endTimeUsec - now;
    //std::cout << "travel time" << travelTimeUsec << " " << traveltime << std::endl;
    //distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;
    distance_cm = travelTimeUsec * 340.29 / 20000;

    return distance_cm;
} 

void HC_SR04::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH );
    endTimeUsec = micros();
}



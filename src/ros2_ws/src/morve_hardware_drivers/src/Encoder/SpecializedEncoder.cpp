/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.cpp   *
 **********************************/

#include <stdexcept>
#include <iostream>
#include <numbers>
#include <string>
#include <numeric>
#include "morve_hardware_drivers/Encoder/SpecializedEncoder.h"

using namespace std::numbers;

SpecializedEncoder::SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m) : Encoder(signal_A, signal_B){
    this->pulses_per_rotation = pulses_per_rotation;
    this->diameter_m = diameter_m;
    distance_per_pulse = (diameter_m * pi) / pulses_per_rotation;
    coeficient_window = {1};
    meassured_delta_pulses = {0};
}

SpecializedEncoder::SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m, size_t window_size) : Encoder(signal_A, signal_B){
    this->pulses_per_rotation = pulses_per_rotation;
    this->diameter_m = diameter_m;
    distance_per_pulse = (diameter_m * pi) / pulses_per_rotation;
    meassured_delta_pulses = std::list<double>(window_size);
}

void SpecializedEncoder::update(){
    RefreshDeltaPulses();
    meassured_delta_pulses.pop_front();
    meassured_delta_pulses.push_back(GetDeltaPulses());
    
    average_delta_pulses = std::accumulate(meassured_delta_pulses.begin(), meassured_delta_pulses.end(), 0);
    average_delta_pulses /= meassured_delta_pulses.size();
}

double SpecializedEncoder::GetDeltaDistance() const{
    return GetDeltaPulses() * distance_per_pulse;
}

double SpecializedEncoder::GetAngularSpeed(double delta_time) const{
    return (2 * pi * average_delta_pulses) / (pulses_per_rotation * delta_time);
}

double SpecializedEncoder::GetSpeed(double delta_time) const{
    return (pi * average_delta_pulses * diameter_m) / (pulses_per_rotation * delta_time);
}
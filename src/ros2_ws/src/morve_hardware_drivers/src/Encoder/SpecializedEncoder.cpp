/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.cpp   *
 **********************************/


#include <numbers>
#include "morve_hardware_drivers/Encoder/SpecializedEncoder.h"

using namespace std::numbers;

SpecializedEncoder::SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m) : Encoder(signal_A, signal_B){
    this->pulses_per_rotation = pulses_per_rotation;
    this->diameter_m = diameter_m;
    distance_per_pulse = (diameter_m * pi) / pulses_per_rotation;
}

double SpecializedEncoder::GetDeltaDistance() const{
    return GetDeltaPulses() * distance_per_pulse;
}

double SpecializedEncoder::GetAngularSpeed(double delta_time) const{
    return (2 * pi * GetDeltaPulses()) / (pulses_per_rotation * delta_time);
}

double SpecializedEncoder::GetSpeed(double delta_time) const{
    return (pi * GetDeltaPulses() * diameter_m) / (pulses_per_rotation * delta_time);
}
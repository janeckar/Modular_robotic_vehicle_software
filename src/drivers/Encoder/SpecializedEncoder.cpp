/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.cpp   *
 **********************************/


#include <numbers>
#include "SpecializedEncoder.h"

using namespace std::numbers;

SpecializedEncoder::SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_cm) : Encoder(signal_A, signal_B){
    distance_per_pulse = ((diameter_cm / 100) * pi) / pulses_per_rotation;
}

double SpecializedEncoder::GetDeltaDistance_m(){
    return GetDeltaPulses() * distance_per_pulse;
}
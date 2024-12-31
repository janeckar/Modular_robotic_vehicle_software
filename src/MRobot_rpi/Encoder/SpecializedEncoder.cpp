/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.cpp   *
 **********************************/



//#include <cmath>
#include <numbers>
#include "SpecializedEncoder.h"

using namespace std::numbers;

SpecializedEncoder::SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_cm) : Encoder(signal_A, signal_B){
    double i = pi;
}

SpecializedEncoder::~SpecializedEncoder() {
}

double SpecializedEncoder::GetDeltaSpeed(){
    return 0.0;
}
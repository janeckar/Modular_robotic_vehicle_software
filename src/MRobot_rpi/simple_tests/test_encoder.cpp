/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 29. 12. 2024             *
 * file: test_encoder.cpp         *
 **********************************/

#include <iostream>
#include <wiringPi.h>
#include "Encoder.h"

const int pinA_FLM = 7;
const int pinB_FLM = 5; // FLM ... front left motor
const int pinA_RLM = 6;
const int pinB_RLM = 12; // RLM ... rear left motor
const int pinA_FRM = 16;
const int pinB_FRM = 13; // RRM ... front right motor
const int pinA_RRM = 20;
const int pinB_RRM = 19; // RRM ... rear right motor

int main(int argc, char* argv[]){
    if(wiringPiSetupGpio() < 0)
        return -1;

    Encoder encoder_FLM(pinA_FLM, pinB_FLM);
    Encoder encoder_RLM(pinA_RLM, pinB_RLM);
    Encoder encoder_FRM(pinA_FRM, pinB_FRM);
    Encoder encoder_RRM(pinA_RRM, pinB_RRM);


    while(true){
        std::cout << "delta_FLM: " << encoder_FLM.GetDeltaPulses() 
                  << "delta_RLM: " << encoder_RLM.GetDeltaPulses() 
                  << "delta_FRM: " << encoder_FRM.GetDeltaPulses() 
                  << "delta_RRM: " << encoder_RRM.GetDeltaPulses() << std::endl;
        delay(100);
    }

    return 0;
}
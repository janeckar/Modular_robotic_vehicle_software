/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 29. 12. 2024             *
 * file: test_encoder.cpp         *
 **********************************/

#include <iostream>
#include <wiringPi.h>
#include "morve_hardware_drivers/Encoder/SpecializedEncoder.h"

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

    SpecializedEncoder encoder_FLM(pinA_FLM, pinB_FLM, 265, 0.065);
    SpecializedEncoder encoder_RLM(pinA_RLM, pinB_RLM, 265, 0.065);
    SpecializedEncoder encoder_FRM(pinA_FRM, pinB_FRM, 265, 0.065);
    SpecializedEncoder encoder_RRM(pinA_RRM, pinB_RRM, 265, 0.065);
    // experimentálně bylo zjištěno 265.3 pulsu za otocku pro RRM
    //                              265.35 pro FRM
    //                              265.35 pro FLM
    //                              265.19


    while(true){
        encoder_FLM.RefreshDeltaPulses();
        encoder_RLM.RefreshDeltaPulses();
        encoder_FRM.RefreshDeltaPulses();
        encoder_RRM.RefreshDeltaPulses();

        std::cout << "delta_FLM: " << encoder_FLM.GetDeltaPulses() 
                  << " delta_RLM: " << encoder_RLM.GetDeltaPulses() 
                  << " delta_FRM: " << encoder_FRM.GetDeltaPulses() 
                  << " delta_RRM: " << encoder_RRM.GetDeltaPulses() << std::endl;
        
        std::cout << "delta_FLM: " << encoder_FLM.GetSpeed(0.1) // period 100 ms
                  << " delta_RLM: " << encoder_RLM.GetSpeed(0.1) 
                  << " delta_FRM: " << encoder_FRM.GetSpeed(0.1) 
                  << " delta_RRM: " << encoder_RRM.GetSpeed(0.1) << std::endl;
        // std::cout << "delta_FLM: "  << encoder_FLM.GetCount() 
        //           << " delta_RLM: " << encoder_RLM.GetCount() 
        //           << " delta_FRM: " << encoder_FRM.GetCount() 
        //           << " delta_RRM: " << encoder_RRM.GetCount() 
        //                             << std::endl;
        // std::cout << "delta_FLM: " << encoder_FLM.GetDeltaDistance_m() 
        //           << " delta_RLM: " << encoder_RLM.GetDeltaDistance_m() 
        //           << " delta_FRM: " << encoder_FRM.GetDeltaDistance_m() 
        //           << " delta_RRM: " << encoder_RRM.GetDeltaDistance_m() << std::endl;
        delay(100);
    }

    return 0;
}
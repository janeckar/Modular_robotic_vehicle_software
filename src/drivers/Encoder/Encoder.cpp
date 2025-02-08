/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 28. 12. 2024             *
 * file: Encoder.cpp              *
 **********************************/

#include <exception>
#include <stdexcept>
#include <climits>
#include <wiringPi.h>
#include "Encoder.h"

#include <iostream>

std::size_t Encoder::obj_cnt = 0;
volatile uint32_t Encoder::cnt[MAX_NUM_Encoders] = {0};
std::array<int, Encoder::MAX_NUM_Encoders> Encoder::pin_A = {0};
std::array<int, Encoder::MAX_NUM_Encoders> Encoder::pin_B = {0};

Encoder::Encoder(int signal_A, int signal_B){
    if(obj_cnt >= MAX_NUM_Encoders){
        throw std::runtime_error("Reached maximum number of Encoders.");
    }
    obj_num = obj_cnt;
    pin_A.at(obj_num) = signal_A;
    pin_B.at(obj_num) = signal_B;
    pinMode(pin_A.at(obj_num), INPUT);
    pinMode(pin_B.at(obj_num), INPUT);
    
    setup_interrupt(obj_num);

    obj_cnt++;
}

Encoder::~Encoder(){
    obj_cnt--;
}

void Encoder::setup_interrupt(uint64_t interrupt_num){
    switch(interrupt_num){
        case Num1:
            wiringPiISR(pin_A.at(Num1), INT_EDGE_RISING, signal_A1_interrupt);
            break;
        case Num2:
            wiringPiISR(pin_A.at(Num2), INT_EDGE_RISING, signal_A2_interrupt);
            break;
        case Num3:
            wiringPiISR(pin_A.at(Num3), INT_EDGE_RISING, signal_A3_interrupt);
            break;
        case Num4:
            wiringPiISR(pin_A.at(Num4), INT_EDGE_RISING, signal_A4_interrupt);
            break;
        default:
            std::string msg = "Interrupt function number " + std::to_string(interrupt_num) + "not existing.";
            throw std::runtime_error(msg);
    }
}

void Encoder::signal_A1_interrupt(){
    if(digitalRead(pin_B.at(Num1)) == LOW){
        cnt[Num1]++;
    } else{
        cnt[Num1]--;
    }
}

void Encoder::signal_A2_interrupt(){
    if(digitalRead(pin_B.at(Num2)) == LOW){
        cnt[Num2]++;
    } else{
        cnt[Num2]--;
    }
}

void Encoder::signal_A3_interrupt(){
    if(digitalRead(pin_B.at(Num3)) == LOW){
        cnt[Num3]++;
    } else{
        cnt[Num3]--;
    }
}

void Encoder::signal_A4_interrupt(){
    if(digitalRead(pin_B.at(Num4)) == LOW){
        cnt[Num4]++;
    } else{
        cnt[Num4]--;
    }
}

uint32_t Encoder::GetCount(){
    return cnt[obj_num];   
}

int64_t Encoder::GetDeltaPulses(){
    // cnt and cnt_old number are unsigned int 32bit numbers
    int64_t delta = cnt[obj_num] - cnt_old[obj_num];
    if(delta > INT32_MAX){
        delta -= UINT32_RANGE;
    } else if(delta < INT32_MIN){
        delta += UINT32_RANGE;
    }
    cnt_old[obj_num] = cnt[obj_num];
    return delta;
}

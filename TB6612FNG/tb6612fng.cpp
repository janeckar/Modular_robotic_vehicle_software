/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 4. 12. 2024       *
 * file: tb6612fng.cpp     *
 ***************************/

#include <wiringPi.h>
#include <algorithm>

#include <iostream>

#include "tb6612fng.h"

TB6612FNG::TB6612FNG(int pwm, int in1, int in2){
    pwm_pin = pwm;
    in1_pin = in1;
    in2_pin = in2;
    pinMode(pwm_pin, PWM_OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    pwmWrite(pwm_pin, 0);
}

void TB6612FNG::set_power(int power){
    power = std::clamp(power, -MAX_PWM_VAL, MAX_PWM_VAL);
    std::cout << "duty_cycle: " << power << std::endl;
    pwmWrite(pwm_pin, std::abs(power));
    if(power < 0){
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
    } else{
        digitalWrite(in2_pin, LOW);
        digitalWrite(in1_pin, HIGH);
    }
}

void TB6612FNG::coast(){
    pwmWrite(pwm_pin, MAX_PWM_VAL);
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
}
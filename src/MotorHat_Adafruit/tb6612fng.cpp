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

TB6612FNG::~TB6612FNG(){
    pinMode(pwm_pin, INPUT);
    pinMode(in1_pin, INPUT);
    pinMode(in2_pin, INPUT);
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

// TB6612FNG_I2C

TB6612FNG_I2C::TB6612FNG_I2C(int address, int pwm, int in1, int in2) : pwmModule{address} {
    pwm_pin = pwm;
    in1_pin = in1;
    in2_pin = in2;
    pwmModule.Set_pwm_frequency(MAX_FREQUENCY);
}

TB6612FNG_I2C::~TB6612FNG_I2C() = default;

void TB6612FNG_I2C::set_power(int power){
    power = std::clamp(power, -PWM_RESOLUTION, PWM_RESOLUTION);
    std::cout << "duty_cycle: " << power << std::endl;
    
    pwmModule.Write_pwm_led(pwm_pin, 0, std::abs(power)); // TODO maybe add parameter for rising edge time
    if(power < 0){
        pwmModule.Turn_off_led(in1_pin);
        pwmModule.Turn_on_led(in2_pin);
    } else{
        pwmModule.Turn_off_led(in2_pin);
        pwmModule.Turn_on_led(in1_pin);
    }
}

void TB6612FNG_I2C::coast(){
    pwmModule.Turn_on_led(pwm_pin);
    pwmModule.Turn_off_led(in1_pin);
    pwmModule.Turn_off_led(in2_pin);
}

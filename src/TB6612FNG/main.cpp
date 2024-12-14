/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date of creation: 4. 12. 2024  *
 **********************************/

#include <iostream>
#include <wiringPi.h>
#include "tb6612fng.h"

using namespace std;

const int PWM_PIN = 12; // broadcom pinout
const int INA1 = 5;
const int INA2 = 6;

int main(int argc, char** args){
    // Setups the WiringPi to use Broadcom pinout (in compliance with raspberry pi Gpio pins)
    if (wiringPiSetupGpio() == -1)
        return -1;

    TB6612FNG motor(PWM_PIN, INA1, INA2);

    motor.set_power(150);
    delay(5000);
    motor.set_power(200);
    delay(5000);

    for(int i = 200; i <= MAX_PWM_VAL; i++){
        motor.set_power(i);
        delay(10);
    }
    
    motor.coast();
    delay(1000);

    for(int i = 0; i >= -MAX_PWM_VAL; i--){
        motor.set_power(i);
        delay(10);
    }

    motor.coast();
    delay(1000);

    return 0;
}
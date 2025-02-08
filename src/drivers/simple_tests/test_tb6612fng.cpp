/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date of creation: 4. 12. 2024  *
 **********************************/

#include <iostream>
#include <wiringPi.h>
#include <signal.h>
#include "../MotorHat_Adafruit/tb6612fng.h"

using namespace std;

const int PWM_PIN = 12; // broadcom pinout
const int INA1 = 5;
const int INA2 = 6;

void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   pinMode(PWM_PIN, INPUT);
   pinMode(INA1, INPUT);
   pinMode(INA2, INPUT);
   // i can use bool flag that the sigint was received and then skip all in program
   // but now it would be stupid
   // Terminate program
   exit(signum);
}

int main(int argc, char** args){
    // Setups the WiringPi to use Broadcom pinout (in compliance with raspberry pi Gpio pins)
    if (wiringPiSetupGpio() == -1)
        return -1;

    signal(SIGINT, signal_callback_handler);

    TB6612FNG motor(PWM_PIN, INA1, INA2);

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
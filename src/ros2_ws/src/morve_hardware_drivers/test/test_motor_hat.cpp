/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 18. 12. 2024             *
 **********************************/

#include <iostream>
#include <wiringPi.h>
#include <signal.h>
#include <morve_hardware_drivers/MotorHat_Adafruit/motor_hat.h>

using namespace std;

bool wasSigint = false;
const int MOTOR_HAT_ADDR = 0x60;

void signal_callback_handler(int signum) {
    cout << "Caught signal " << signum << endl;
    wasSigint = true;
}

int main(int argc, char** args){
    // Setups the WiringPi to use Broadcom pinout (in compliance with raspberry pi Gpio pins)
    if (wiringPiSetupGpio() == -1)
        return -1;

    signal(SIGINT, signal_callback_handler);

    motor_hat motorhat(MOTOR_HAT_ADDR);

    for(int i = 500; i <= 1000 && !wasSigint; i++){
        motorhat.motor1.set_power(i);
        motorhat.motor2.set_power(i);
        motorhat.motor3.set_power(i);
        motorhat.motor4.set_power(i);
        delay(1);
    }
    
    motorhat.motor1.coast();
    motorhat.motor2.coast();
    motorhat.motor3.coast();
    motorhat.motor4.coast();
    for(int i = 0; i < 1000 && !wasSigint; i++)
        delay(1);

    for(int i = -500; i >= -1000 && !wasSigint; i--){
        motorhat.motor1.set_power(i);
        motorhat.motor2.set_power(i);
        motorhat.motor3.set_power(i);
        motorhat.motor4.set_power(i);
        delay(1);
    }

    motorhat.motor1.coast();
    motorhat.motor2.coast();
    motorhat.motor3.coast();
    motorhat.motor4.coast();
    for(int i = 0; i < 1000 && !wasSigint; i++)
        delay(1);

    return 0;
}
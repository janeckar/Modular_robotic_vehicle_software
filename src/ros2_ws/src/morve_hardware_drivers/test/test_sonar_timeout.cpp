#include <iostream>
#include <wiringPi.h>
#include "morve_hardware_drivers/Sonar/HC_SR04.h"

using namespace std;

// Broadcom pinout right now
const int trigger = 22;  // wPi ..
const int echo = 23;     // wPi .. 



int main() {
    // Setups the WiringPi to use Broadcom pinout (in compliance with raspberry pi Gpio pins)
    if (wiringPiSetupGpio() == -1)
        return -1;

    HC_SR04 sonar(trigger, echo);

    while(1){
        cout << "Distance is " << sonar.Distance(300) << " cm." << endl;
        cout << "Sensor timeout is " << sonar.MeassureSensorTimeout(300) << "us\n";
        delay(1000);
    }

    return 0;
}

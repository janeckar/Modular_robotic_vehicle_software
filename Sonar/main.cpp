#include <iostream>
#include <wiringPi.h>
#include "HC_SR04.h"

using namespace std;

// Broadcom pinout right now
const int trigger = 18;  // wPi .. 1
const int echo = 24;     // wPi .. 5


int main() {
    if (wiringPiSetupGpio() == -1)
        return -1;

    HC_SR04 sonar(trigger, echo);

    cout << "Hello, World!" << std::endl;

    while(1){
        cout << "Distance is " << sonar.Distance(10000000) << " cm." << endl;
        delay(1000);
    }

    return 0;
}

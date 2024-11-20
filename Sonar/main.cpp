#include <iostream>
#include <wiringPi.h>
#include "HC_SR04.h"

using namespace std;

int main() {
    if (wiringPiSetup() == -1)
        return -1;

    HC_SR04 sonar(1, 5);

    cout << "Hello, World!" << std::endl;

    while(1){
        cout << "Distance is " << sonar.Distance(10000000) << " cm." << endl;
        delay(1000);
    }

    return 0;
}

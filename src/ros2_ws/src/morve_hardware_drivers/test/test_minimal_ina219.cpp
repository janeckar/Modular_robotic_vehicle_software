/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz.cz *
 * date: 28. 3. 2025               *
 * file: minimal_ina219.cpp        *
 ***********************************/

#include <wiringPi.h>
#include <iostream>
#include "morve_hardware_drivers/Battery_Voltage/minimal_ina219.hpp"

bool wasSigint = false;
const int INA219_ADDR = 0x40;

using namespace std;

void signal_callback_handler(int signum) {
    cout << "Caught signal " << signum << endl;
    wasSigint = true;
}

int main(int /*argc*/, char** /*argv*/){
    minimal_ina219 ina219(INA219_ADDR, ina219_constants::adc_modes::ADC_MODE_128_SAMPLES);

    while(!wasSigint){
        cout << "BUS voltage: " << ina219.read_bus_voltage() << " V\n";
        delay(1000);
    }
}
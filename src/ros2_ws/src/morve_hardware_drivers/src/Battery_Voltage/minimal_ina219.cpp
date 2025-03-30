/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz.cz *
 * date: 28. 3. 2025               *
 * file: minimal_ina219.cpp        *
 ***********************************/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <format>
#include <unistd.h>
#include <iostream>
#include "morve_hardware_drivers/Battery_Voltage/minimal_ina219.hpp"

using namespace ina219_constants;

// private

int minimal_ina219::read_16bit_reg(int reg){
    int data = wiringPiI2CReadReg16(device_fd, reg);
    if(data < 0){
        return data;
    }
    // wiring pi expects that the i2c devices comunicates in little endian and it transforms the value automatically to big endian effectivelly swapping the right order of bytes XD
    // transform data from little endian to big endian
    return (data << 8 | data >> 8) & 0xFFFF;
}

int minimal_ina219::write_16bit_reg(int reg, int data){
    // wiring pi expects that the i2c devices comunicates in little endian and it transforms the value automatically to big endian effectivelly swapping the right order of bytes XD
    // transform data to little endian from big endian
    data = (data << 8 | data >> 8) & 0xFFFF;
    return wiringPiI2CWriteReg16(device_fd, reg, data);
}

void minimal_ina219::wait_for_measurement(ina219_constants::adc_modes adc_mode){
    delayMicroseconds(MTIME_DELAY);
    switch(adc_mode){
        case ADC_MODE_9BIT:
            delayMicroseconds(MTIME_9BIT);
            break;
        case ADC_MODE_10BIT:
            delayMicroseconds(MTIME_10BIT);
            break;
        case ADC_MODE_11BIT:
            delayMicroseconds(MTIME_11BIT);
            break;
        case ADC_MODE_12BIT:
            delayMicroseconds(MTIME_12BIT);
            break;
        case ADC_MODE_2_SAMPLES:
            delayMicroseconds(MTIME_2_SAMPLES);
            break;
        case ADC_MODE_4_SAMPLES:
            delayMicroseconds(MTIME_4_SAMPLES);
            break;
        case ADC_MODE_8_SAMPLES:
            delayMicroseconds(MTIME_8_SAMPLES);
            break;
        case ADC_MODE_16_SAMPLES:
            delayMicroseconds(MTIME_16_SAMPLES);
            break;
        case ADC_MODE_32_SAMPLES:
            delayMicroseconds(MTIME_32_SAMPLES);
            break;
        case ADC_MODE_64_SAMPLES:
            delayMicroseconds(MTIME_64_SAMPLES);
            break;
        case ADC_MODE_128_SAMPLES:
            delayMicroseconds(MTIME_128_SAMPLES);
            break;
    }
}


// public

minimal_ina219::minimal_ina219(int i2c_addr, adc_modes precision){
    i2c_address = i2c_addr;
    device_fd = wiringPiI2CSetup(i2c_addr);
    measurement_precision = precision;
    if(device_fd < 0){
        const auto msg = std::format("Could not open i2c device on address 0x{:x}.", i2c_addr);
        throw std::system_error{errno, std::system_category(), msg};
    }
    
    power_up();

    if((configuration = read_16bit_reg(CONFIGURATION_REG)) < 0){
        close(device_fd);
        const auto msg = std::format("Failed to read 16 bit configuration register of device ina219 on address 0x{:x}.", i2c_addr);
        throw std::system_error{errno, std::system_category(), msg};
    }

    configuration &= ~MODE_MASK; // clears mode bits
    configuration |= MODE_BUS_TRIGGER; // sets trigger mode with bus measurement
    configuration &= ~ADC_MODE_MASK; // clear adc_mode
    configuration |= measurement_precision; // set adc precision or samples

    if(write_16bit_reg(CONFIGURATION_REG, configuration) < 0){
        close(device_fd);
        const auto msg = std::format("Failed to write 16 bit configuration register of device ina219 on address 0x{:x}.", i2c_addr);
        throw std::system_error{errno, std::system_category(), msg};
    }
}   

void minimal_ina219::sleep(){
    int sleep_conf = configuration & ~(MODE_MASK);
    write_16bit_reg(CONFIGURATION_REG, sleep_conf);
}

void minimal_ina219::power_up(){
    int res = write_16bit_reg(CONFIGURATION_REG, RST);
    if(res < 0){
        const auto msg = std::format("Failed to start the ina219 on address 0x{:x}", i2c_address);
        throw std::system_error(errno, std::system_category(), msg);
    }
    delayMicroseconds(TIME_POWER_ON);
    write_16bit_reg(CONFIGURATION_REG, configuration);
}

double minimal_ina219::read_bus_voltage(){
    write_16bit_reg(CONFIGURATION_REG, configuration); // send trigger and wakes up the ina219 chip

    wait_for_measurement(measurement_precision);
    int bus_reg_value = read_16bit_reg(BUS_VOLTAGE_REG);
    if(bus_reg_value < 0){
        const auto msg = std::format("Failed to read the bus voltage with ina219 on address 0x{:x}", i2c_address);
        throw std::system_error(errno, std::system_category(), msg);
    }
    // int ready = bus_reg_value & CNVR; // it should be always ready

    bus_reg_value = bus_reg_value >> BUS_VOLTAGE_OFFSET; // after this only the bus voltage value remains
    
    return bus_reg_value * BUS_VOLTAGE_UNIT / 1000.0;
}
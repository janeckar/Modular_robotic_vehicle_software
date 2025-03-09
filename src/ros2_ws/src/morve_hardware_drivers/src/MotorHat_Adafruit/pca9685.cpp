/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 17. 12. 2024      *
 * file: pca9685.cpp       *
 ***************************/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <system_error>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include "morve_hardware_drivers/MotorHat_Adafruit/pca9685.h"

pca9685::pca9685(int i2c_addr){
    device_fd = wiringPiI2CSetup(i2c_addr); // connect to pca9685
    if(device_fd < 0){
        const auto msg = std::format("Could not open i2c device on address {:x}.", i2c_addr);
        throw std::system_error{errno, std::system_category(), msg};
    }
    int res = wiringPiI2CWriteReg8(device_fd, MODE2, OUTDRV); // setting with no pull up resistor
    res |= wiringPiI2CWriteReg8(device_fd, MODE1, ALLCALL); // waking clock and setting allcall address flag
    delay(1); //ms
    // write default frequency
    res |= Set_pwm_frequency(DEFAULT_FREQUENCY);
    res |= Clear_all_pwm();
    if(res < 0){
        pca9685::~pca9685();
        throw std::system_error{errno, std::system_category(), "Failed to initialize PCA9685."};
    }
}

pca9685::~pca9685(){
    if(device_fd >= 0){
        Clear_all_pwm();
        wiringPiI2CWriteReg8(device_fd, MODE1, SLEEP);
        close(device_fd);
    }
}

int pca9685::Set_pwm_frequency(int freq){
    int clipped_freq = std::clamp(freq, MIN_FREQUENCY, MAX_FREQUENCY);
    int pre_scale_val = static_cast<int>(std::round((double)CLK_FREQUENCY / (PWM_RESOLUTION * clipped_freq) - 1));

    int oldmode1 = wiringPiI2CReadReg8(device_fd, MODE1);
    if (oldmode1 < 0){
        return oldmode1;
    }
    uint8_t oldmode1_u8 = static_cast<uint8_t>(oldmode1);
    uint8_t newmode1_u8 = (oldmode1_u8 & ~RESTART) | SLEEP; // turn off internal clk
    int res = wiringPiI2CWriteReg8(device_fd, MODE1, newmode1_u8);
    res |= wiringPiI2CWriteReg8(device_fd, PRE_SCALE, pre_scale_val);
    res |= wiringPiI2CWriteReg8(device_fd, MODE1, oldmode1_u8); // turn on internall clk
    delay(1); //ms
    res |= wiringPiI2CWriteReg8(device_fd, MODE1, oldmode1_u8 | RESTART);
    return res;
}

int pca9685::Write_pwm_led(uint8_t LED_num, uint16_t rising_edge_time,  uint16_t duty_cycle){
    if(LED_num > 15)
        return -1;

    switch(duty_cycle){
        case 0:
            return Turn_off_led(LED_num);
            break;
        case PWM_RESOLUTION:
            return Turn_on_led(LED_num);
            break;
        default:
            int offset = LED_SHIFT * LED_num;
            uint16_t off_time = (rising_edge_time + duty_cycle) % PWM_RESOLUTION;
            int res = wiringPiI2CWriteReg8(device_fd, LED0_ON_L + offset, rising_edge_time & 0xFF);
            res |= wiringPiI2CWriteReg8(device_fd, LED0_ON_H + offset, rising_edge_time >> 8);
            res |= wiringPiI2CWriteReg8(device_fd, LED0_OFF_L + offset, off_time & 0xFF);
            res |= wiringPiI2CWriteReg8(device_fd, LED0_OFF_H + offset, off_time >> 8);
            return res;
    } 
}

int pca9685::Write_pwm_all(uint16_t rising_edge_time, uint16_t duty_cycle){
    int res = wiringPiI2CWriteReg8(device_fd, ALL_LED_ON_L, rising_edge_time & 0xFF);
    res |= wiringPiI2CWriteReg8(device_fd, ALL_LED_ON_H, rising_edge_time >> 8);

    uint16_t off_time = (rising_edge_time + duty_cycle) % PWM_RESOLUTION;
    res |= wiringPiI2CWriteReg8(device_fd, ALL_LED_OFF_L, off_time & 0xFF);
    res |= wiringPiI2CWriteReg8(device_fd, ALL_LED_OFF_H, off_time >> 8);
    return res;
}

int pca9685::Turn_on_led(uint8_t LED_num){
    int offset = LED_SHIFT * LED_num;
    int res = wiringPiI2CWriteReg8(device_fd, LED0_OFF_H + offset, 0); // 12-th bit must be zero
    res |= wiringPiI2CWriteReg8(device_fd, LED0_ON_H + offset, LED_ON);
    return res;
}

int pca9685::Turn_off_led(uint8_t LED_num){
    int offset = LED_SHIFT * LED_num;
    return wiringPiI2CWriteReg8(device_fd, LED0_OFF_H + offset, LED_OFF);
}

int pca9685::Clear_all_pwm(){
    // fastest way to turn off all outputs is to write a logic 1 to bit 4 in register the ALL_LED_OFF_H
    return wiringPiI2CWriteReg8(device_fd, ALL_LED_OFF_H, LED_OFF);
}

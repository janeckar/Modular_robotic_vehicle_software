/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 19. 12. 2024      *
 * file: motor_hat.cpp     *
 ***************************/

#include "morve_hardware_drivers/MotorHat_Adafruit/motor_hat.h"

motor_hat::motor_hat(int i2c_address) : motors{TB6612FNG_I2C{i2c_address, PWM_M1, IN1_M1, IN2_M1},
                                               TB6612FNG_I2C{i2c_address, PWM_M2, IN1_M2, IN2_M2},
                                               TB6612FNG_I2C{i2c_address, PWM_M3, IN1_M3, IN2_M3},
                                               TB6612FNG_I2C{i2c_address, PWM_M4, IN1_M4, IN2_M4}}
{}

motor_hat::~motor_hat() = default;
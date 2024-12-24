/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 19. 12. 2024      *
 * file: motor_hat.cpp     *
 ***************************/

#include "motor_hat.h"

motor_hat::motor_hat(int i2c_address) : motor1{i2c_address, PWM_M1, IN1_M1, IN2_M1}
                                      , motor2{i2c_address, PWM_M2, IN1_M2, IN2_M2}
                                      , motor3{i2c_address, PWM_M3, IN1_M3, IN2_M3}
                                      , motor4{i2c_address, PWM_M4, IN1_M4, IN2_M4}
{}

motor_hat::~motor_hat() = default;
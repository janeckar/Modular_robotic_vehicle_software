/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 19. 12. 2024      *
 * file: motor_hat.h       *
 ***************************/

#ifndef MOTOR_HAT_H
#define MOTOR_HAT_H

#include <array>
#include "tb6612fng.h"


class motor_hat{
  private:
    static const int PWM_M1 = 8;
    static const int IN1_M1 = 10;
    static const int IN2_M1 = 9;
    static const int PWM_M2 = 13;
    static const int IN1_M2 = 11;
    static const int IN2_M2 = 12;
    static const int PWM_M3 = 2;
    static const int IN1_M3 = 4;
    static const int IN2_M3 = 3;
    static const int PWM_M4 = 7;
    static const int IN1_M4 = 5;
    static const int IN2_M4 = 6;

  public:
    std::array<TB6612FNG_I2C, 4> motors;

    /**
     * @par address I2C address of PCA9685 on motor HAT
     */
    motor_hat(int address);
    ~motor_hat();
};

#endif // MOTOR_HAT_H
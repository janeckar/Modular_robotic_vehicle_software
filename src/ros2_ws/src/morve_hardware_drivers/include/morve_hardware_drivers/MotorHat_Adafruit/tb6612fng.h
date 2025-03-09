/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 4. 12. 2024       *
 * file: tb6612fng.h       *
 ***************************/

#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "pca9685.h"

const int MAX_PWM_VAL = 1023; // raspberry pi PWM

class TB6612FNG {
  private:
    int pwm_pin;
    int in1_pin;
    int in2_pin;

  public:
    
    /**
     * @brief creates object for Motor control with H-bridge TB6612FNG
     * @param pwm pin used for PWM
     * @param in1 pin used as Output
     * @param in2 pin used as Output
     */
    TB6612FNG(int pwm, int in1, int in2);
    ~TB6612FNG();
    
    /**
     * @param power dutycycle with sign for direction of rotation, (-1024,1024), 0 .. value stands for brake
     */
    void set_power(int power);
    
    /**
     * @brief stop mode from tb6612fng datasheet
     */
    void coast();
};

class TB6612FNG_I2C {
  /**
   * @note This class is intended to use with the TB6612FNG and PCA9685
   */
  private:
    int pwm_pin;
    int in1_pin;
    int in2_pin;
    pca9685 pwmModule;

  public:
    /**
     * @brief creates object for Motor control with H-bridge TB6612FNG via I2C BUS
     * @param address i2c adress of PCA9685 chip
     * @param pwm PCA9685 pin used for PWM
     * @param in1 PCA9685 pin used as Output
     * @param in2 PCA9685 pin used as Output
     */
    TB6612FNG_I2C(int address, int pwm, int in1, int in2); // pins of PCA9685
    ~TB6612FNG_I2C();

    /**
     * @brief sets the rotation (sign of power) and relative speed of rotation 
     * @param power dutycycle with sign for direction of rotation, (-4096,4096), 0 .. value stands for brake
     */
    void set_power(int power);

    /**
     * @brief stop mode from tb6612fng datasheet
     */
    void coast();

    void clear_all_pwm_outputs();
};

#endif // TB6612FNG_H
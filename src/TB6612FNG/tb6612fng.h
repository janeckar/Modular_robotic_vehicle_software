/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 4. 12. 2024       *
 * file: tb6612fng.h       *
 ***************************/

#ifndef TB6612FNG_H
#define TB6612FNG_H

const int MAX_PWM_VAL = 1023; // raspberry pi PWM

class TB6612FNG {
  private:
    int pwm_pin;
    int in1_pin;
    int in2_pin;

  public:
    
    /**
     * @brief creates object for Motor control with H-bridge TB6612FNG
     * @par pwm pin used for PWM
     * @par in1 pin used as Output
     * @par in2 pin used as Output
     */
    TB6612FNG(int pwm, int in1, int in2);
    ~TB6612FNG();
    
    /**
     * @par int power, dutycycle with sign for direction of rotation, (-1024,1024), 0 .. value stands for brake
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
  public:
    /**
     * @brief creates object for Motor control with H-bridge TB6612FNG via I2C BUS
     * @par address i2c adress of PCA9685 chip
     * @par pwm PCA9685 pin used for PWM
     * @par in1 PCA9685 pin used as Output
     * @par in2 PCA9685 pin used as Output
     */
    TB6612FNG_I2C(int address, int pwm, int in1, int in2); // pins of PCA9685
    ~TB6612FNG_I2C();

    /**
     * @par int power, dutycycle with sign for direction of rotation, (-1024,1024), 0 .. value stands for brake
     */
    void set_power(int power);

    /**
     * @brief stop mode from tb6612fng datasheet
     */
    void coast();
};

#endif // TB6612FNG_H
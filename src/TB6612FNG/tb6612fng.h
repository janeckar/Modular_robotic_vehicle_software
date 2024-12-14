/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 4. 12. 2024       *
 * file: tb6612fng.h       *
 ***************************/

#ifndef TB6612FNG_H
#define TB6612FNG_H

const int MAX_PWM_VAL = 1023;

class TB6612FNG {
  private:
    /*
      three pins for single motor to drive
      four modes
      pwm duty cycle change

      choice of i2c comunication
    */
    int pwm_pin;
    int in1_pin;
    int in2_pin;

  public:
    /**
     * @par address i2c adress
     */
    //TB6612FNG(int address ); // pins of PCA9685
    
    TB6612FNG(int pwm, int in1, int in2);

    //void rotate_cw(int dutycycle);
    //void rotate_ccw(int dutycycle);
    //void set_dutycycle(int dutycycle);
    
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
/***************************
 * author: Radek Janečka   *
 * email: xjanec34@vut.cz  *
 * date: 17. 12. 2024      *
 * file: pca9685.h         *
 ***************************/

#ifndef PCA9685_H
#define PCA9685_H

#include <cstdint>

// register addresses
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t SUBADDR1 = 0x02;
constexpr uint8_t SUBADDR2 = 0x03;
constexpr uint8_t SUBADDR3 = 0x04;
constexpr uint8_t ALLCALLADDR = 0x05;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t LED0_ON_H = 0x07;
constexpr uint8_t LED0_OFF_L = 0x08;
constexpr uint8_t LED0_OFF_H = 0x09;
constexpr uint8_t ALL_LED_ON_L = 0xFA;
constexpr uint8_t ALL_LED_ON_H = 0xFB;
constexpr uint8_t ALL_LED_OFF_L = 0xFC;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;
constexpr uint8_t PRE_SCALE = 0xFE;

// led shift
const int LED_SHIFT = 4;

// led off/on instant
constexpr uint8_t LED_ON = 0x10; // write to H half of ON register
constexpr uint8_t LED_OFF = 0x10; // write to H half of OFF register

// flags for register MODE1
constexpr uint8_t ALLCALL = 0x01;
constexpr uint8_t SUB1 = 0x08; // enables sub call address 1
constexpr uint8_t SUB2 = 0x04; // enables sub call address 2
constexpr uint8_t SUB3 = 0x02; // enables sub call address 3
constexpr uint8_t SLEEP = 0x10;
constexpr uint8_t AI = 0x20; // autoincrement
constexpr uint8_t EXTCLK = 0x40;
constexpr uint8_t RESTART = 0x80;

// flags for register MODE2
constexpr uint8_t OUTDRV = 0x04; // no external pull up resistor required

// frequency
constexpr int MIN_FREQUENCY = 24; // Hz
constexpr int DEFAULT_FREQUENCY = 200; // Hz
constexpr int MAX_FREQUENCY = 1526; // Hz
constexpr int CLK_FREQUENCY = 25'000'000; // 25 MHz
constexpr int PWM_RESOLUTION = 4096; // 12 bit

class pca9685 {
  private:
    int device_fd;
    int all_fd;
  public:
    /**
     * @par i2c_addr I2C address of peripheral
     * @throw system_error
     */
    explicit pca9685(int i2c_addr);
    ~pca9685();

    /**
     * @par freq in range <24; 1526> Hz
     */
    int Set_pwm_frequency(int freq);

    /**
     * @par LED_num can be <0; 15>
     * @par duty_cycle can be in range <0; 4095>
     */
    int Write_pwm_led(uint8_t LED_num, uint16_t rising_edge_time,  uint16_t duty_cycle); 
    int Write_pwm_all(uint16_t rising_edge_time, uint16_t duty_cycle);
    int Turn_on_led(uint8_t LED_num);
    int Turn_off_led(uint8_t LED_num);
    int Clear_all_pwm();

};

#endif // PCA9685_H
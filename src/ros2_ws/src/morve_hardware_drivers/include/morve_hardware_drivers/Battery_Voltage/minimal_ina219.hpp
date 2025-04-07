/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 28. 3. 2025               *
 * file: minimal_ina219.hpp        *
 ***********************************/

#ifndef MORVE_HARDWARE_DRIVERS_MINIMAL_INA219_HPP
#define MORVE_HARDWARE_DRIVERS_MINIMAL_INA219_HPP

#include <unistd.h>

namespace ina219_constants{
  // to access the registers first needs to be written register pointer
  /**** register addresses ****/
  constexpr int16_t CONFIGURATION_REG = 0x00; // Read/Write 2x8 bit
  constexpr int16_t SHUNT_VOLTAGE_REG = 0x01; // Read only 2x8 bit
  constexpr int16_t BUS_VOLTAGE_REG = 0x02; // Read only 2x8 bit
  constexpr int16_t POWER_REG = 0x03; // Read only 2x8 bit
  constexpr int16_t CURRENT_REG = 0x04; // Read only 2x8 bit
  constexpr int16_t CALIBRATION_REG = 0x05; // Read/Write 2x8 bit
  
  /**** bits for configuration register ****/
  constexpr int16_t MODE1 = 0x01; // if MODE2 or MODE3 is set then it changes the mode from trigger to continuous, else if not then it disables ADC 
  constexpr int16_t MODE2 = 0x02; // enables bus voltage measurement
  constexpr int16_t MODE3 = 0x04; // enables shunt voltage measurement
  // when all MODEn bits are 0 then the chip is powered down
  constexpr int16_t RST = 0x80;
  constexpr int16_t BRNG = 0x2000;
  constexpr int16_t MODE_BUS_CONTINUOUS = MODE1 | MODE2;
  constexpr int16_t MODE_BUS_TRIGGER = MODE2;
  // used for clearing the MODES
  constexpr int16_t MODE_MASK = MODE1 | MODE2 | MODE3;

  /**** Resolution of the voltage measurement ****/
  enum adc_modes{
    ADC_MODE_9BIT = 0x0,
    ADC_MODE_10BIT = 0x1,
    ADC_MODE_11BIT = 0x2,
    ADC_MODE_12BIT = 0x3,
    ADC_MODE_2_SAMPLES = 0x9,
    ADC_MODE_4_SAMPLES = 0xA,
    ADC_MODE_8_SAMPLES = 0xB,
    ADC_MODE_16_SAMPLES = 0xC,
    ADC_MODE_32_SAMPLES = 0xD,
    ADC_MODE_64_SAMPLES = 0xE,
    ADC_MODE_128_SAMPLES = 0xF
  };
  constexpr int16_t ADC_MODE_MASK = 0xF;
  constexpr int SHUNT_ADC_MODE_OFFSET = 3;
  constexpr int BUS_ADC_MODE_OFFSET = 7;


  /**** BUS voltage register flags and offsets ****/
  constexpr int16_t OVF = 0x01; // overflow, indicates when the power or current calculations are out of range
  /* - conversion ready flag, tells if new values are ready to be read from registers
     - clears when read power register or new mode was written to configuration reg */
  constexpr int16_t CNVR = 0x02;
  constexpr int BUS_VOLTAGE_OFFSET = 3;
  constexpr int BUS_VOLTAGE_UNIT = 4; // mV

  // measure time
  constexpr useconds_t MTIME_DELAY = 100; // add this time to the measure time to be sure that the value is in register
  constexpr useconds_t MTIME_9BIT = 84;
  constexpr useconds_t MTIME_10BIT = 148;
  constexpr useconds_t MTIME_11BIT = 276;
  constexpr useconds_t MTIME_12BIT = 532; // (default) measure time for 12 bit precision
  constexpr useconds_t MTIME_2_SAMPLES = 1'060;
  constexpr useconds_t MTIME_4_SAMPLES = 2'130;
  constexpr useconds_t MTIME_8_SAMPLES = 4'260;
  constexpr useconds_t MTIME_16_SAMPLES = 8'510;
  constexpr useconds_t MTIME_32_SAMPLES = 17'020;
  constexpr useconds_t MTIME_64_SAMPLES = 34'050;
  constexpr useconds_t MTIME_128_SAMPLES = 68'100;
  constexpr useconds_t TIME_POWER_ON = 35'000; // delay needed before the ina219 is ready

} // namespace ina219_constants 

// focus for trigger mode only, then it will be much easier

class minimal_ina219{
  private:
    int i2c_address;
    int device_fd;
    int configuration; // using only trigger configuration
    ina219_constants::adc_modes measurement_precision;
    // read and write of data with the transformation of the values, swap bits, wiringPi expects data be transfered in little endian but the opposite is true
    int read_16bit_reg(int reg);
    int write_16bit_reg(int reg, int data);
    void wait_for_measurement(ina219_constants::adc_modes adc_mode);
  public:
    explicit minimal_ina219(int i2c_addr, ina219_constants::adc_modes precision);
    ~minimal_ina219(){close(device_fd);};
    void sleep();
    void power_up();
    double read_bus_voltage();
};

#endif // MORVE_HARDWARE_DRIVERS_MINIMAL_INA219_HPP
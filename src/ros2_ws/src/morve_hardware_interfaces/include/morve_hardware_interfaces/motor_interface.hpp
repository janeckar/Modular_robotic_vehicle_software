/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 12. 2. 2025              *
 * file: motor_interface.hpp      *
 **********************************/

#ifndef MORVE_HARDWARE_INTERFACES_MOTOR_INTERFACE
#define MORVE_HARDWARE_INTERFACES_MOTOR_INTERFACE

#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "morve_hardware_drivers/MotorHat_Adafruit/motor_hat.h"
#include "morve_hardware_drivers/Encoder/SpecializedEncoder.h"

namespace morve_hardware_interfaces {
  class motor_interface : public hardware_interface::SystemInterface {
    private:
      int motor_hat_i2c_addr;
      int motor_output_mapping;
      int encoder_line_A_gpio_pin; // broadcom pin number
      int encoder_line_B_gpio_pin; // broadcom pin number
      int encoder_pulses_per_rotation;
      double wheel_diameter_metres;

      std::shared_ptr<motor_hat> motorhat_;
      std::shared_ptr<SpecializedEncoder> wheel_encoder_;

      double joint_effort_commands_;
      double joint_velocity_states_;

      /**
       * @brief finds substring in string with ignoring the case of chars
       * 
       * @param substr substring o be found in string
       * @param str original string
       * @return int position of the substring in string, -1 if the substring wasn't found
       * @throw 
       */
      static bool starts_with_ic(const std::string & substr, const std::string & str);

      /**
       * @brief extracts number from gpio string
       * 
       * @param gpio_str string representation of the gpio pin in broadcom notation
       * @return int representation of the gpio pin in broadcom notation
       * @throws std::invalid_argument,
       *         std::range_error
       */
      static int parse_gpio(std::string gpio_str);

      /**
       * @brief extracts number from motor mapping string
       * 
       * @param motor_mapping_str string mapping to the output of the motor hat which should be used
       * @return int access integer to the motor hat
       * @throws std::invalid_argument,
       *         std::range_error
       */
      static int parse_motor_num(std::string motor_mapping_str);

    public:
      motor_interface();
      CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
      CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
      hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override; // todo replace with valid msgtype???
      // it should use PWM, min -1024 max 1024
      // effort interface message it could use

  };
}

#endif // MORVE_HARDWARE_INTERFACES_MOTOR_INTERFACE
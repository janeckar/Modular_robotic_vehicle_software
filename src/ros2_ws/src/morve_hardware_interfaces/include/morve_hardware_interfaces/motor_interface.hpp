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

      

    public:
      CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
      CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
      //CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      //CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

      //std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      //std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
      hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override; // todo replace with valid msgtype???
      // it should use PWM, min -1024 max 1024
      // effort interface message it could use

  };
}

#endif // MORVE_HARDWARE_INTERFACES_MOTOR_INTERFACE
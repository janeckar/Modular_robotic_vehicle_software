/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 12. 2. 2025              *
 * file: motor_interface.hpp      *
 **********************************/

#ifndef MOVE4_HARDWARE_INTERFACES_MOTOR_INTERFACE
#define MOVE4_HARDWARE_INTERFACES_MOTOR_INTERFACE

#include "hardware_interface/actuator_interface.hpp" // maybe system if i will want to have there also encoders

namespace morve_hardware_interfaces {
  class motor_interface : public hardware_interface::ActuatorInterface {
    public:
      motor_interface();
      CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
      CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
      CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
      CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
      CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
      CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
      CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
      hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override; // todo replace with valid msgtype???
      // it should use PWM, min -1024 max 1024

  };
}

#endif // MOVE4_HARDWARE_INTERFACES_MOTOR_INTERFACE
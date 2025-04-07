/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 3. 4. 2025                *
 * file: pwm_pid_controller.hpp    *
 ***********************************/

#ifndef PWM_PID_CONTROLLER__PWM_PID_CONTROLLER_HPP
#define PWM_PID_CONTROLLER__PWM_PID_CONTROLLER_HPP

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

namespace pwm_pid_controller
{
   
class PwmPidController : public controller_interface::ChainableControllerInterface
{
  public:
    PwmPidController() = default;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;
    using ControllerMeasuredStateMsg = control_msgs::msg::MultiDOFCommand;
    using ControllerModeSrvType = std_srvs::srv::SetBool;
    using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;
  protected:
    // std::shared_ptr<pid_controller::ParamListener> param_listener_;
    // pid_controller::Params params_;

    std::vector<std::string> reference_and_state_dof_names_;
    size_t dof_;
    std::vector<double> measured_state_values_;

    // using PidPtr = std::shared_ptr<control_toolbox::PidROS>;
    // std::vector<PidPtr> pids_;

    
};


} // namespace pwm_pid_controller

#endif // PWM_PID_CONTROLLER__PWM_PID_CONTROLLER_HPP
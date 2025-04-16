// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Daniel Azanov, Dr. Denis
//


/***********************************
 * modified by: Radek Janečka      *
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
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "pwm_pid_controller/pid_regulator.hpp"
#include "pwm_pid_controller/pwm_pid_controller_parameters.hpp"

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
    std::shared_ptr<pwm_pid_controller::ParamListener> param_listener_;
    pwm_pid_controller::Params params_;

    std::vector<std::string> reference_and_state_dof_names_;
    size_t dof_;
    std::vector<double> measured_state_values_;

    std::vector<std::shared_ptr<pid_regulator>> pids_;
    
    // internal method
    void update_parameters();
    controller_interface::CallbackReturn configure_parameters();
  
    // Command subscribers
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
    
    // State subscribers
    rclcpp::Subscription<ControllerMeasuredStateMsg>::SharedPtr measured_state_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerMeasuredStateMsg>> measured_state_;

    // rclcpp::Service<ControllerModeSrvType>::SharedPtr set_feedforward_control_service_;
    // realtime_tools::RealtimeBuffer<feedforward_mode_type> control_mode_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
    
    // Controler State publisher
    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    // override methods from ChainableControllerInterface
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

  private:
    // callback for topic interface
    void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};


} // namespace pwm_pid_controller

#endif // PWM_PID_CONTROLLER__PWM_PID_CONTROLLER_HPP
/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 12. 2. 2025              *
 * file: motor_interface.cpp      *
 **********************************/

#include <algorithm>
#include <string>
#include <cstring>
#include <memory>

#include "morve_hardware_interfaces/motor_interface.hpp"
#include "morve_hardware_interfaces/parsing_functions.hpp"
#include "wiringPi.h" // some problems with compiling if the wiringPi include is before morve_hardware_interfaces
#include "rclcpp/rclcpp.hpp"

namespace morve_hardware_interfaces
{

hardware_interface::CallbackReturn motor_interface::on_init(
  const hardware_interface::HardwareInfo & hardware_info) 
{
  // first check if successfuly loaded urdf file
  if( hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // then load parameters
  // i2c address of pca9685fng or motor hat
  try{
    motor_hat_i2c_addr = morve_hardware_interfaces::verbose_stoi(info_.hardware_parameters["motor_hat_i2c_addr"], 0, 16);
  } catch (const std::invalid_argument & e){
    RCLCPP_FATAL(
      get_logger(), "Parameter motor_hat_i2c_addr = '%s' is not a number in hex notation.", 
      info_.hardware_parameters["motor_hat_i2c_addr"].c_str());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::range_error & e){
    RCLCPP_FATAL(get_logger(), "Parameter motor_hat_i2c_addr: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // load joint info
  if(info_.joints.size() == 0 || info_.joints.size() > 4){
    RCLCPP_ERROR(get_logger(), "Expects one to four joints but got %ld.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for(auto joint : info_.joints){
    // which output to choose from the motor hat mapping
    try{
      motor_output_mappings[joint.name] = morve_hardware_interfaces::parse_motor_num(joint.parameters["motor_hat_output_mapping"]);
    } catch (const std::invalid_argument & e){
      RCLCPP_FATAL(get_logger(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    } catch(const std::range_error & e){
      RCLCPP_FATAL(get_logger(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // parameters for encoder
    try{
      encoder_line_A_gpio_pins[joint.name] = morve_hardware_interfaces::parse_gpio(joint.parameters["encoder_line_A_gpio_pin"]);
      encoder_line_B_gpio_pins[joint.name] = morve_hardware_interfaces::parse_gpio(joint.parameters["encoder_line_B_gpio_pin"]);
      encoder_pulses_per_rotation[joint.name] = morve_hardware_interfaces::verbose_stoi(joint.parameters["encoder_pulses_per_rotation"]);
      wheel_diameter_metres[joint.name] = std::stod(joint.parameters["wheel_diameter_metres"]);
      if((window_sizes[joint.name] = morve_hardware_interfaces::verbose_stoi(joint.parameters["window_size"])) < 1){
        RCLCPP_FATAL(get_logger(), "Window size of %s cannot be zero!", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } catch (std::invalid_argument & e){
      RCLCPP_FATAL(get_logger(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    } catch (std::range_error & e){
      RCLCPP_FATAL(get_logger(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // command interface
    if(joint.command_interfaces.size() != 1){
      RCLCPP_ERROR(get_logger(), "Joint '%s' have %zu command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if(joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT){
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' has '%s' as command interface. '%s' expected",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // state interface
    if(joint.state_interfaces.size() < 1 || joint.state_interfaces.size() > 2){
      RCLCPP_ERROR(get_logger(), "Joint '%s' have %zu state interfaces. 1-2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // parsing state_interfaces velocity and optional position
    if(joint.state_interfaces.size() == 1){
      if(joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY){
        RCLCPP_ERROR(
          get_logger(), "Joint '%s' has '%s' as state interface. '%s' expected",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else{
      if(joint.state_interfaces[0].name == joint.state_interfaces[1].name){
        RCLCPP_ERROR(
          get_logger(), "Joint '%s' has two identical '%s' state interfaces! '%s' and optional '%s' expected",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION
        );
        return hardware_interface::CallbackReturn::ERROR;
      }
      for(auto state_interface : joint.state_interfaces){
        if(state_interface.name != hardware_interface::HW_IF_VELOCITY && 
          state_interface.name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_ERROR(
            get_logger(), "Joint '%s' has '%s' as state interface. '%s' and optional '%s' expected",
            joint.name.c_str(), state_interface.name.c_str(),
            hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION
          );
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn motor_interface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring motor and encoder hardware resources...");
  // configures hardware
  if (int result = wiringPiSetupGpio() == -1){
    std::string msg = "WiringPi setup failed with error code: " + std::to_string(result);
    RCLCPP_FATAL(get_logger(), msg.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  try{
    auto temp_motorhat = std::make_shared<motor_hat>(motor_hat_i2c_addr);
    for(auto joint : info_.joints){
      auto temp_wheel_encoder = std::make_shared<SpecializedEncoder>(
        encoder_line_A_gpio_pins[joint.name], encoder_line_B_gpio_pins[joint.name], encoder_pulses_per_rotation[joint.name],
        wheel_diameter_metres[joint.name], window_sizes[joint.name]);
      
      wheel_encoders_[joint.name] = std::move(temp_wheel_encoder);
    }
    motorhat_ = std::move(temp_motorhat);
  } catch (const std::system_error & e) {
    RCLCPP_ERROR(get_logger(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::bad_alloc & e){
    RCLCPP_ERROR(get_logger(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  motorhat_->motors[0].clear_all_pwm_outputs();
  for(auto joint : info_.joints){
    wheel_encoders_[joint.name]->ResetCounters();
  }

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn motor_interface::on_cleanup(
  const rclcpp_lifecycle::State & /*pevious_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning motor and encoder hardware resources...");
  motorhat_->motors[0].clear_all_pwm_outputs();
  motorhat_->~motor_hat();
  motorhat_ = nullptr;

  // to be restarted it must be turned off
  for(auto joint : info_.joints){
    wheel_encoders_[joint.name]->~SpecializedEncoder();
    wheel_encoders_[joint.name] = nullptr;
  }
  RCLCPP_INFO(get_logger(), "Successfullly cleaned!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn motor_interface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{ 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn motor_interface::on_activate(
  const rclcpp_lifecycle::State & /*pevious_state*/)
{
  for(auto [name, descr] : joint_command_interfaces_){
    RCLCPP_INFO(
      get_logger(), "Activating wheel joint %s.", name.c_str());
  }
  
  for(auto joint : info_.joints){
    wheel_encoders_[joint.name]->ResetCounters();
    motorhat_->motors[motor_output_mappings[joint.name]].coast();
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn motor_interface::on_deactivate(
  const rclcpp_lifecycle::State & /*pevious_state*/)
{
  for(auto [name, descr] : joint_command_interfaces_){
    RCLCPP_INFO(
      get_logger(), "Deactivating wheel joint %s.", name.c_str());
  }
  motorhat_->motors[0].clear_all_pwm_outputs();
  for(auto joint : info_.joints){
    wheel_encoders_[joint.name]->ResetCounters();
  }
  
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

/*std::vector<hardware_interface::CommandInterface> motor_interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for(const auto & joint : info_.joints){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &joint_effort_commands_));
  }

  return command_interfaces;
}

std::vector<hardware_interface::StateInterface> motor_interface::export_state_interfaces()
{

}*/

hardware_interface::return_type motor_interface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      wheel_encoders_[descr.get_prefix_name()]->update();
      auto speed = wheel_encoders_[descr.get_prefix_name()]->GetAngularSpeed(period.seconds()); // precision can be compromised
      set_state(name, speed);
    } else { // position interface
      auto position = wheel_encoders_[descr.get_prefix_name()]->GetAngularDistance();
      set_state(name, position);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type motor_interface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // get effort, pwm value

  for(const auto & [name, descr] : joint_command_interfaces_)
  {
    if(descr.get_interface_name() == hardware_interface::HW_IF_EFFORT){
      int pwm = static_cast<int>(get_command(name));
      motorhat_->motors[motor_output_mappings[descr.get_prefix_name()]].set_power(pwm);
    }
  }
  
  return hardware_interface::return_type::OK;
}

} // namespace morve_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(morve_hardware_interfaces::motor_interface, hardware_interface::SystemInterface)
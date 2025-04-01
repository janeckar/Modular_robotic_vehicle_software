/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz.cz *
 * date: 31. 3. 2025               *
 * file: battery_voltage.cpp       *
 ***********************************/

#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "morve_hardware_drivers/Battery_Voltage/minimal_ina219.hpp"
#include "battery_voltage/battery_voltage.hpp"
#include "morve_interfaces/msg/battery_status.hpp"

constexpr int ina219_address = 0x40; 

void VoltagePublisher::define_parameters()
{
  // low_voltage_level
  auto param_desc_low_voltage = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_low_voltage.description = "Voltage threshold when it will be sent that the battery is low on power.";

  this->declare_parameter("low_voltage_level", 6.0, param_desc_low_voltage);

  // ina219_address
  auto param_desc_ina219_addr = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_ina219_addr.description = "I2C address of sensor INA219 in hex format. Needs to be set before running the node.";

  this->declare_parameter("ina219_address", ina219_address, param_desc_ina219_addr);
}

void VoltagePublisher::measure_voltage_timed()
{
  auto message = morve_interfaces::msg::BatteryStatus();
  message.voltage = ina219_->read_bus_voltage();  
  message.battery_low = message.voltage < this->get_parameter("low_voltage_level").as_double();
  this->publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Publishing battery status.");
}

VoltagePublisher::VoltagePublisher() : Node("voltage_publisher")
{
  define_parameters();
  try{
    ina219_ = std::make_shared<minimal_ina219>(get_parameter("ina219_address").as_int(), ina219_constants::adc_modes::ADC_MODE_128_SAMPLES);
  } catch(const std::system_error & e){
    RCLCPP_FATAL(this->get_logger(), e.what());
    throw;
  }
  publisher_ = this->create_publisher<morve_interfaces::msg::BatteryStatus>("/battery", rclcpp::QoS(1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&VoltagePublisher::measure_voltage_timed, this));
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<VoltagePublisher>());
  } catch(const std::system_error & e){}
  rclcpp::shutdown();
  return 0;
}

/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz.cz *
 * date: 1. 4. 2025                *
 * file: battery_voltage.hpp       *
 ***********************************/

#ifndef BATTERY_VOLTAGE_BATTERY_VOLTAGE_HPP
#define BATTERY_VOLTAGE_BATTERY_VOLTAGE_HPP

#include <rclcpp/rclcpp.hpp>
#include "morve_interfaces/msg/battery_status.hpp"
#include "morve_hardware_drivers/Battery_Voltage/minimal_ina219.hpp"

class VoltagePublisher : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<morve_interfaces::msg::BatteryStatus>::SharedPtr publisher_;
    std::shared_ptr<minimal_ina219> ina219_;
    void measure_voltage_timed();
    void define_parameters();

  public:
    VoltagePublisher();
};

#endif // BATTERY_VOLTAGE_BATTERY_VOLTAGE_HPP
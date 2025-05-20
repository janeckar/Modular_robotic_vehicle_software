/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 12. 5. 2025               *
 * file: sonar_node.hpp            *
 ***********************************/

#ifndef SONAR_DISTANCE_SONAR_NODE_HPP
#define SONAR_DISTANCE_SONAR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/range.hpp"
#include "morve_hardware_drivers/Sonar/HC_SR04.h"
#include "sonar_distance/sonar_node_parameters.hpp"

namespace sonar_distance{

class SonarPublisher : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    std::shared_ptr<HC_SR04> sonar_;
    sensor_msgs::msg::Range range_msg_;
    std::shared_ptr<sonar_distance::ParamListener> param_listener_;
    sonar_distance::Params params_;
    int measure_time_;

    void measure_callback();

  public:
    SonarPublisher();
    void init();
};

} // namespace sonar_distance

#endif // SONAR_DISTANCE_SONAR_NODE_HPP
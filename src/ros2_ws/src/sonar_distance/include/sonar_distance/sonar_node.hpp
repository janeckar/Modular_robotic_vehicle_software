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
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    std::shared_ptr<HC_SR04> sonar_;
    sensor_msgs::msg::Range range_msg_;
    std::shared_ptr<sonar_distance::ParamListener> param_listener_;
    sonar_distance::Params params_;

    void measure_callback();

  public:
    SonarPublisher();
    
    /**
     * @brief Initializes the sonar node, sets up parameters, and prepares the publisher.
     * 
     * This method should be called after creating an instance of SonarPublisher.
     * It initializes the sonar hardware, sets up the publisher, and prepares the parameters.
     */
    void init();
    void update_params();

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> get_publisher() const { return publisher_; }
    std::shared_ptr<HC_SR04> get_sonar() const { return sonar_; }
    const sonar_distance::Params & get_params() const { return params_; }
    const sensor_msgs::msg::Range & get_range_msg() const { return range_msg_; }
};

} // namespace sonar_distance

#endif // SONAR_DISTANCE_SONAR_NODE_HPP
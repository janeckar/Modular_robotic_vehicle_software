/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 12. 5. 2025               *
 * file: sonar_node.cpp            *
 ***********************************/

#include <memory>
#include <thread>
#include <string>
#include <wiringPi.h>
#include "sonar_distance/sonar_node.hpp"
#include "morve_hardware_drivers/hardware_error.h"
#include "sonar_distance/sonar_node_parameters.hpp"

namespace sonar_distance{

SonarPublisher::SonarPublisher() : rclcpp::Node("sonar") {}

void SonarPublisher::init(){
   try
  {
    param_listener_ = std::make_shared<ParamListener>(shared_from_this());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    throw;
  }

  // inicialize template range message
  range_msg_.radiation_type = range_msg_.ULTRASOUND;
  range_msg_.field_of_view = params_.field_of_view;
  range_msg_.min_range = params_.min_range;
  range_msg_.max_range = params_.max_range;
  measure_time_ = params_.measurement_time;
  
  if (int result = wiringPiSetupGpio() == -1){
    std::string msg = "WiringPi setup failed with error code: " + std::to_string(result);
    RCLCPP_FATAL(get_logger(), msg.c_str());
    throw hardware_error(msg);
  }

  try{
  sonar_ = std::make_shared<HC_SR04>(params_.trigger_gpio, params_.echo_gpio);
  } catch (const hardware_error & e){
    RCLCPP_ERROR(get_logger(), "Failed construction of sonar with exception: %s", e.what());
    throw;
  }

  // creates publisher publishing to the topic named as the node name 
  std::string topic_name = "/" + std::string(this->get_name());
  publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic_name, rclcpp::QoS(1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(params_.measurement_time), bind(&SonarPublisher::measure_callback, this));
}

void SonarPublisher::measure_callback(){
  range_msg_.range = sonar_->read_distance();
  range_msg_.header.stamp = this->now();
  range_msg_.header.frame_id = this->get_name();

  sonar_->request_distance();

  publisher_->publish(range_msg_);
}

} // namespace sonar_distance


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try{
    auto node = std::make_shared<sonar_distance::SonarPublisher>();
    node->init();
    rclcpp::spin(node);
  } catch(const hardware_error & e){

  } catch(const std::exception & e){}
  
  rclcpp::shutdown();
  return 0;
}


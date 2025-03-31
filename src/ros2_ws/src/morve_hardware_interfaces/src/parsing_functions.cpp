/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 14. 3. 2025              *
 * file: parsing_functions.cpp    *
 **********************************/

#include <stdexcept>
#include "morve_hardware_interfaces/parsing_functions.hpp"

namespace morve_hardware_interfaces
{

int verbose_stoi(const std::string &__str, std::size_t *__idx, int __base){
  int value;
  try{
    value = std::stoi(__str, __idx, __base);
  } catch (const std::range_error & e){
    std::string msg = "stoi : range_error in string number: " + __str;
    throw std::range_error(msg);
  } catch(const std::invalid_argument & e){
    std::string msg = "stoi : invalid_argument in string number: " + __str;
  }
  return value;
}

bool starts_with_ic(const std::string & str, const std::string & substr){
  if(substr.length() > str.length()){
    return false;
  }

  int substr_len = substr.length();

  for(int sub_i = 0; sub_i < substr_len; sub_i++){
    if(std::toupper(str.at(sub_i)) != std::toupper(substr.at(sub_i))){
      break;
    } else if(substr_len -1 == sub_i){
      return true;
    }
  }

  return false;
}
  
int parse_gpio(std::string gpio_str){
  if(starts_with_ic(gpio_str, "gpio")){
    gpio_str = gpio_str.substr(4);
  }
  // TODO check range of gpio
  return verbose_stoi(gpio_str);
}

int parse_motor_num(std::string motor_mapping_str){
  std::string motor_mapping_number;
  if(starts_with_ic(motor_mapping_str, "motor")){
    motor_mapping_str = motor_mapping_str.substr(5);
  }
  // checks range of mapping
  int motor_mapping = verbose_stoi(motor_mapping_str);
  if(motor_mapping < 0 || motor_mapping > 3){
    std::string msg = "Motor hat output mapping number " + motor_mapping_str + " is not in range <0; 3>.";
    throw std::range_error(msg);
  }
  return motor_mapping;
}

} // namespace morve_hardware_interfaces
/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 4. 4. 2025                *
 * file: pid_regulator.cpp         *
 ***********************************/

#include <cmath>
#include "pwm_pid_controller/pid_regulator.hpp"

bool pid_regulator::is_close(double a, double b, double tolerance){
  return std::fabs(a - b) < tolerance;
}

void pid_regulator::initialize_pid_regulator(double Kp, double Ki, double Kd, double i_min, double i_max, double out_dead_zone_low, double out_dead_zone_high){
  gains.Kp = Kp;
  gains.Ki = Ki;
  gains.Kd = Kd;
  gains.i_min = i_min;
  gains.i_max = i_max;
  gains.out_dead_zone_low = out_dead_zone_low;
  gains.out_dead_zone_high = out_dead_zone_high;
}

double pid_regulator::compute_command(double error, rclcpp::Duration dt){
  double first_term, second_term, third_term;

  errors[2] = errors[1];
  errors[1] = errors[0];
  errors[0] = error; // error = target - state

  double derivative_part = gains.Kd / dt.seconds();

  coefficients[0] = gains.Kp + gains.Ki * dt.seconds() + derivative_part;
  first_term =  coefficients[0] * errors[0];

  coefficients[1] = -gains.Kp - 2 * derivative_part; 
  second_term = coefficients[1] * errors[1];

  coefficients[2] = derivative_part;
  third_term = coefficients[2] * errors[2];

  last_out_cmd = last_out_cmd + first_term + second_term + third_term;

  last_out_cmd = std::min(gains.i_max, std::max(gains.i_min, last_out_cmd)); // apply bounds to output command

  return last_out_cmd;
}

double pid_regulator::compute_command_dead_zone(double target, double state, rclcpp::Duration dt){
  compute_command(target - state, dt);
  
  if(!is_close(last_target, target)){ // check if last_target differs from new target
    int sign_last_target = sign<double>(last_target);
    int sign_target = sign<double>(target);

    // changing the output command also changes the integral part
    if (is_close(target, 0.0)){
      set_output_cmd(0.0);
    } else if(sign_last_target < sign_target || (is_close(last_target, 0.0) && target > 0.0)){
      set_output_cmd(gains.out_dead_zone_high);
    } else if(sign_last_target > sign_target){
      set_output_cmd(gains.out_dead_zone_low);
    } 
  }
  // set current target as previous target
  last_target = target;

  return get_output_cmd();
}

void pid_regulator::reset(){
  last_target = 0.0;
  last_out_cmd = 0.0;
  for(size_t i = 0; i < pid::SIZE_OF_ERRORS_OR_COEFFICIENTS; i++){
    errors[i] = 0.0;
    coefficients[i] = 0.0;
  }
}

bool pid_regulator::set_gains_from_parameter(pwm_pid_controller::Params::Gains::MapDofNames param_gains){
  if(param_gains.i_clamp_max < param_gains.i_clamp_min ||
     param_gains.out_dead_zone_high < param_gains.out_dead_zone_low){
    return false;
  }
  gains.Kp = param_gains.p;
  gains.Ki = param_gains.i;
  gains.Kd = param_gains.d;
  gains.i_max = param_gains.i_clamp_max;
  gains.i_min = param_gains.i_clamp_min;
  gains.out_dead_zone_high = param_gains.out_dead_zone_high;
  gains.out_dead_zone_low = param_gains.out_dead_zone_low;

  return true;
}
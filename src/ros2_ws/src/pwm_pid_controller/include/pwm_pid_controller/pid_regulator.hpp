/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 4. 4. 2025                *
 * file: pid_regulator.hpp         *
 ***********************************/

#ifndef PWM_PID_REGULATOR__PID_REGULATOR_HPP
#define PWM_PID_REGULATOR__PID_REGULATOR_HPP

#include <list>
#include "rclcpp/rclcpp.hpp"
#include "pwm_pid_controller/pwm_pid_controller_parameters.hpp"

namespace pid
{
  constexpr size_t SIZE_OF_ERRORS_OR_COEFFICIENTS = 3;
};

class pid_regulator{
  protected:
    // gains
    struct Gains {
    double Kp = 0.0; // proportional gain
    double Ki = 0.0; // integral gain
    double Kd = 0.0; // derivative gain
    double i_max = 0.0; // maximal value of the integral error when to clamp
    double i_min = 0.0; // minimal value of the integral error when to clamp the value
    double out_dead_zone_high = 0.0; // high dead zone border from which the regulated value should start changing 
    double out_dead_zone_low = 0.0; // low dead zone border, when the cmd value is less than this the state should start changing
    };    

    Gains gains;
    double last_target = 0.0;
    double last_out_cmd = 0.0; // last out command signal
    double errors[pid::SIZE_OF_ERRORS_OR_COEFFICIENTS];
    double coefficients[pid::SIZE_OF_ERRORS_OR_COEFFICIENTS];

    void set_output_cmd(double new_output_cmd) {last_out_cmd = new_output_cmd;};

  private:
    template <typename T>
    int sign(T val){
      return (val >= T(0)) ? 1 : -1;
    }
    
    bool is_close(double a, double b, double tolerance = 1e-6);

  public:

    pid_regulator() = default;

    void initialize_pid_regulator(double Kp, double Ki, double Kd, double i_min, double i_max, double out_dead_zone_low = 0.0, double out_dead_zone_high = 0.0);
    double compute_command(double error, rclcpp::Duration dt);
    double compute_command_dead_zone(double target, double state, rclcpp::Duration dt);
    void reset();
    bool set_gains_from_parameter(pwm_pid_controller::Params::Gains::MapDofNames param_gains);

    double get_output_cmd() const {return last_out_cmd;};
};

#endif // PWM_PID_REGULATOR__PID_REGULATOR_HPP
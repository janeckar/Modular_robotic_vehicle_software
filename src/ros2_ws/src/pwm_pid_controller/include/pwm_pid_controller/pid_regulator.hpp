/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 4. 4. 2025                *
 * file: pid_regulator.hpp         *
 ***********************************/

#ifndef PWM_PID_REGULATOR__PID_REGULATOR_HPP
#define PWM_PID_REGULATOR__PID_REGULATOR_HPP

class pid_regulator{
  protected:
    // gains
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double i_max = 0.0; // maximal value of the integral error when to clamp
    double i_min = 0.0; // minimal value of the integral error when to clamp the value
    double out_dead_zone_max = 0.0; // 
    double out_dead_zone_min = 0.0;

  public:
    pid_regulator() = default;

    void initialize_pid_regulator(double p, double i, double d, double i_max)
}

#endif // PWM_PID_REGULATOR__PID_REGULATOR_HPP
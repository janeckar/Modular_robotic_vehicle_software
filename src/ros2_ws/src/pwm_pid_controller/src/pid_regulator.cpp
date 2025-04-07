/***********************************
 * author: Radek Janečka           *
 * email: gitjaneckar@seznam.cz    *
 * date: 4. 4. 2025                *
 * file: pid_regulator.cpp         *
 ***********************************/

#include "pwm_pid_controller/pid_regulator.hpp"


00290 double Pid::computeCommand(double error, double error_dot, ros::Duration dt)
00291 {
00292   // Get the gain parameters from the realtime buffer
00293   Gains gains = *gains_buffer_.readFromRT();
00294 
00295   double p_term, d_term, i_term;
00296   p_error_ = error; // this is error = target - state
00297   d_error_ = error_dot;
00298 
00299   if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
00300     return 0.0;
00301 
00302 
00303   // Calculate proportional contribution to command
00304   p_term = gains.p_gain_ * p_error_;
00305 
00306   // Calculate the integral of the position error
00307   i_error_ += dt.toSec() * p_error_;
00308 
00309   // Calculate integral contribution to command
00310   i_term = gains.i_gain_ * i_error_;
00311 
00312   // Limit i_term so that the limit is meaningful in the output
00313   i_term = std::max( gains.i_min_, std::min( i_term, gains.i_max_) );
00314 
00315   // Calculate derivative contribution to command
00316   d_term = gains.d_gain_ * d_error_;
00317 
00318   // Compute the command
00319   cmd_ = p_term + i_term + d_term;
00320 
00321   return cmd_;
00322 }
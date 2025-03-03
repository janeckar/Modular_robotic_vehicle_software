/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 1. 3. 2024               *
 * file: hardware_error.h         *
 **********************************/


#ifndef HARDWARE_ERROR_MORVE_HARDWARE_DRIVERS_H
#define HARDWARE_ERROR_MORVE_HARDWARE_DRIVERS_H

#include <string>

class hardware_error{
  private:
    const std::string & err_msg;
  public:
    hardware_error(const std::string & msg) : err_msg{msg}{};
    virtual ~hardware_error() = default;
    virtual const char* what() const {return err_msg.c_str();};
};

#endif //HARDWARE_ERROR_MORVE_HARDWARE_DRIVERS_H
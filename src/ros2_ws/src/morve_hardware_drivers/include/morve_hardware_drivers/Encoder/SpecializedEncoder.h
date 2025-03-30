/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.h     *
 **********************************/

#ifndef SPECIALIZED_ENCODER_H
#define SPECIALIZED_ENCODER_H

#include <list>
#include "Encoder.h"

class SpecializedEncoder : public Encoder{
  protected:
    double diameter_m;
    int pulses_per_rotation;
    double distance_per_pulse;
    std::vector<double> coeficient_window;
    std::list<double> meassured_delta_pulses;
    double average_delta_pulses;
  public:
    /**
     * @brief Construct a new SpecialEncoder object
     * 
     * @param signal_A number of pin in Broadcom notation
     * @param signal_B number of pin in Broadcom notation
     * @param pulses_per_rotation number of pulses per rotation of encoder
     * @param diameter_m wheel diameter
     * 
     * @throws std::runtime_errorm,
     *         hardware_error
     */
    SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m);
    
    /**
     * @brief Construct a new Specialized Encoder object
     * 
     * @param signal_A number of pin in Broadcom notation
     * @param signal_B number of pin in Broadcom notation
     * @param pulses_per_rotation number of pulses per rotation of encoder
     * @param diameter_m wheel diameter
     * @param coeficient_window number of coeficients determine the size of history messured values stored to calculate the weighted average value
     */
    SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m, size_t window_size);
    
    virtual ~SpecializedEncoder() = default;

    /**
     * @brief call before calling all functions which gets speed or distance data
     * 
     */
    void update();

    /**
     * @brief Gets delta distance in metres
     * 
     * Functionality depends on periodical calls to update()
     * 
     * @return double delta distance in metres
     */
    double GetDeltaDistance() const;

    /**
     * @brief Get the Angular Speed in radians per second
     * 
     * Functionality depends on periodical calls to update()
     * 
     * @param delta_time time in seconds elapsed from last call to update() method
     *
     * @return double radians per second
     */
    double GetAngularSpeed(double delta_time) const;

    /**
     * @brief Get Speed in metres per second
     * 
     * Functionality depends on periodical calls to update()
     * 
     * @param delta_time time elapsed from last call to update() method
     * 
     * @return double metres per second
     */
    double GetSpeed(double delta_time) const;
};

#endif // SPECIALIZED_ENCODER_H
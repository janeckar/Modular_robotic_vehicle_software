/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 30. 12. 2024             *
 * file: SpecializedEncoder.h     *
 **********************************/

#ifndef SPECIALIZED_ENCODER_H
#define SPECIALIZED_ENCODER_H

#include "Encoder.h"

class SpecializedEncoder : public Encoder{
  protected:
    double diameter_m;
    int pulses_per_rotation;
    double delta_period;
    double distance_per_pulse;
  public:
    /**
     * @brief Construct a new SpecialEncoder object
     * 
     * @param signal_A number of pin in Broadcom notation
     * @param signal_B number of pin in Broadcom notation
     * @param pulses_per_rotation number of pulses per rotation of encoder
     * @param diameter_m diameter of wheel with encoder
     * 
     * @throws std::runtime_errorm,
     *         hardware_error
     */
    SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_m);
    virtual ~SpecializedEncoder() = default;

    /**
     * @brief Gets delta distance in metres
     * 
     * Functionality depends on periodical calls to RefreshDeltaPulses()
     * 
     * @return double delta distance in metres
     */
    double GetDeltaDistance() const;

    /**
     * @brief Get the Angular Speed in radians per second
     * 
     * Functionality depends on periodical calls to RefreshDeltaPulses()
     * 
     * @param delta_time time in seconds elapsed from last call to RefreshDeltaPulses() method
     *
     * @return double radians per second
     */
    double GetAngularSpeed(double delta_time) const;

    /**
     * @brief Get Speed in metres per second
     * 
     * Functionality depends on periodical calls to RefreshDeltaPulses()
     * 
     * @param delta_time time elapsed from last call to RefreshDeltaPulses() method
     * 
     * @return double metres per second
     */
    double GetSpeed(double delta_time) const;
};

#endif // SPECIALIZED_ENCODER_H
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
    double distance_per_pulse;
  public:
    SpecializedEncoder(int signal_A, int signal_B, int pulses_per_rotation, double diameter_cm);
    virtual ~SpecializedEncoder();

    double GetDeltaSpeed();
};

#endif // SPECIALIZED_ENCODER_H
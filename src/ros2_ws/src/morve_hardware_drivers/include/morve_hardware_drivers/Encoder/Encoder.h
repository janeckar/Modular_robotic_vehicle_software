/**********************************
 * author: Radek Janečka          *
 * email: xjanec34@vut.cz         *
 * date: 28. 12. 2024             *
 * file: Encoder.h                *
 **********************************/

#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include <array>
#include <string>
#include <functional>

class Encoder{
  protected:
    static constexpr uint64_t UINT32_RANGE = (1ULL << 32);
    static const uint32_t Num1 = 0;
    static const uint32_t Num2 = 1;
    static const uint32_t Num3 = 2;
    static const uint32_t Num4 = 3;
    static const std::size_t MAX_NUM_Encoders = 4;
    static std::size_t obj_cnt;
    uint32_t obj_num;
    static volatile uint32_t cnt[MAX_NUM_Encoders];
    uint32_t cnt_old[MAX_NUM_Encoders] = {0};
    int64_t delta_pulses = 0;
    static std::array<int, MAX_NUM_Encoders> pin_A;
    static std::array<int, MAX_NUM_Encoders> pin_B;
    static void signal_A1_interrupt();
    static void signal_A2_interrupt();
    static void signal_A3_interrupt();
    static void signal_A4_interrupt();

    void setup_interrupt(uint64_t interrupt_num);
  public:
    /**
     * @brief Construct a new Encoder object
     * 
     * @param signal_A number of pin in Broadcom notation
     * @param signal_B number of pin in Broadcom notation
     * 
     * @throws std::runtime_errorm,
     *         hardware_error
     */
    Encoder(int signal_A, int signal_B);

    /**
     * @brief Destroy the Encoder object
     *
     * Resets the pins to the default options
     * 
     */
    virtual ~Encoder();

    /**
     * @brief Get the count on counter of the encoder
     * 
     * @return uint32_t 
     */
    uint32_t GetCount() const;

    /**
     * @brief resets internal counters
     * 
     * It can influence the delta pulses, use only for testing the number of pulses per rotation of motor
     *
     */
    void ResetCounters();

    /**
     * @brief Call this function periodically to refresh delta count
     * 
     * @return int64_t delta count of pulses
     */
    void RefreshDeltaPulses();

    /**
     * @brief Gets delta of pulses which were recorded in delta time after call of RefreshDeltaPulses()
     * 
     * @return int64_t delta count of pulses
     */
    const int64_t & GetDeltaPulses() const {return delta_pulses;};
};

#endif // ENCODER_H
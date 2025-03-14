/**********************************
 * author: Radek Janečka          *
 * email: gitjaneckar@seznam.cz   *
 * date: 14. 3. 2025              *
 * file: parsing_functions.hpp    *
 **********************************/

#ifndef MORVE_HARDWARE_INTERFACES_PARSING_FUNCTIONS
#define MORVE_HARDWARE_INTERFACES_PARSING_FUNCTIONS

#include <string>

namespace morve_hardware_interfaces
{
/**
 * @brief Gives more verbose exceptions
 * 
 * @param __str to be converted to number
 * @param __idx index from which to start converting
 * @param __base number base
 * @return int converted string to integer
 * @throws std::range_error,
 *         std::invalid_argument
 */
int verbose_stoi(const std::string &__str, std::size_t *__idx = (std::size_t *)0, int __base = 10);

/**
 * @brief finds substring in string with ignoring the case of chars
 * 
 * @param substr substring o be found in string
 * @param str original string
 * @return int position of the substring in string, -1 if the substring wasn't found
 * @throw 
 */
bool starts_with_ic(const std::string & str, const std::string & substr);

/**
 * @brief extracts number from gpio string
 * 
 * @param gpio_str string representation of the gpio pin in broadcom notation
 * @return int representation of the gpio pin in broadcom notation
 * @throws std::invalid_argument,
 *         std::range_error
 */
int parse_gpio(std::string gpio_str);

/**
 * @brief extracts number from motor mapping string
 * 
 * @param motor_mapping_str string mapping to the output of the motor hat which should be used
 * @return int access integer to the motor hat
 * @throws std::invalid_argument,
 *         std::range_error
 */
int parse_motor_num(std::string motor_mapping_str);

} // namespace morve_hardware_interfaces

#endif // MORVE_HARDWARE_INTERFACES_PARSING_FUNCTIONS

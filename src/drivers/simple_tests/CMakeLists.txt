# first executable
add_executable(test_motor_hat test_motor_hat.cpp)
target_link_libraries(test_motor_hat PRIVATE MotorHat_Adafruit wiringPi)

# second executable
add_executable(test_tb6612fng test_tb6612fng.cpp ../MotorHat_Adafruit/tb6612fng.cpp ../MotorHat_Adafruit/tb6612fng.h)
target_link_libraries(test_tb6612fng PRIVATE wiringPi tb6612fng)

# third executable
add_executable(test_encoder test_encoder.cpp)
target_link_libraries(test_encoder PUBLIC wiringPi SpecializedEncoder)

add_executable(test_numbers test_numbers.cpp)
cmake_minimum_required(VERSION 3.18)
project(test_motor)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_motor test_tb6612fng.cpp ../MotorHat_Adafruit/tb6612fng.cpp ../MotorHat_Adafruit/tb6612fng.h)

target_link_libraries(test_motor wiringPi)
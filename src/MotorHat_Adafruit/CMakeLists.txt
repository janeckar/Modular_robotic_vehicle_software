cmake_minimum_required(VERSION 3.18)
project(testMotors)

set(CMAKE_CXX_STANDARD 17)

add_executable(testMotors pca9685.cpp pca9685.h main.cpp)

target_link_libraries(testMorors wiringPi)
cmake_minimum_required(VERSION 3.18)
project(test_motor)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_motor main.cpp tb6612fng.cpp tb6612fng.h)

target_link_libraries(test_motor wiringPi)
cmake_minimum_required(VERSION 3.18)
project(Sonar)

set(CMAKE_CXX_STANDARD 17)

add_executable(Sonar HC_SR04.cpp main.cpp
        HC_SR04.cpp)

target_link_libraries(Sonar PRIVATE wiringPi)

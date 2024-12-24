cmake_minimum_required(VERSION 3.18)
project(testMotors)

set(CMAKE_CXX_STANDARD 17)

add_executable(testMotors pca9685.cpp pca9685.h tb6612fng.cpp tb6612fng.h motor_hat.cpp motor_hat.h test_motor_hat.cpp)

#target_sources(testMotors PRIVATE )
target_link_libraries(testMotors wiringPi)

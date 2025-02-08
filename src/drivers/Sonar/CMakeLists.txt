add_executable(Sonar HC_SR04.cpp main.cpp
        HC_SR04.h)

target_link_libraries(Sonar PRIVATE wiringPi)
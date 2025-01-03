cmake_minimum_required(VERSION 3.15)

project(Modular_Robotic_Vehicle VERSION 1.0)

# specify C++ standard
# add_library(compile_flags INTERFACE)
# target_compile_features(compile_flags INTERFACE cxx_std_11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# set the output path for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executables)
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

option(BUILD_RASPBERRYPI "build for raspberrypi specific code" OFF)
option(BUILD_PC "build for main pc, direct robot hardware access not needed" OFF)

add_subdirectory(MRobot_common)

if(BUILD_RASPBERRYPI)
    add_subdirectory(MRobot_rpi)
endif()

if(BUILD_PC)
    add_subdirectory(MRobot_pc)
endif()
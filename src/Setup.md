# How to setup the Modular robotic vehicle
## Introduction to the project

Robotic vehicle consists from several programs comunicating with each other via network. These programs are called modules
and each module has its purposse. For example one of the modules handles motor control.

What is more important that not all of the modules will run on the robot (RaspberryPi) but on the Main PC and will comunicate via Wi-Fi. So we need to compile some programs for RaspberryPi and the other half
for specific Operating System in our case Linux to be exact Ubuntu or Fedora.

## Examples of compilation of the project

1. Example for RaspberryPi
    
    
        mkdir build && cd build
        cmake .. -DBUILD_RPI=ON
        make

2. Example for Linux Desktop

        mkdir build && cd build
        cmake .. -DBUILD_PC
        make (add flag -jN ,*)

    \* replace N with the number of threads, typically you can specify 2x more than cores of cpu

3. Example how to show all available options for cmake
    In build directory

        cmake .. -L
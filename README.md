# Modular_robotic_vehicle_software

## Needed libraries, setup instructions

1. WiringPi

2. ROS2

3. ROS2 packages
    - ros2_control
    - ros2_controllers
    https://control.ros.org/jazzy/doc/getting_started/getting_started.html#building-from-source

4. xacro
    - generating URDF from xacro files 

5. the most useful thing is that i only need to specify the dependencies in the xml files of packages
    and then we can call rosdep in the workspace so it downloads the dependencies and i don't have to specify
    all things which needs to be installed, that can be breakthrough

    ```
        rosdep update --rosdistro=${ROS_DISTRO}
        rosdep install --from-paths src --ignore-src -r -y 
        rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
    ```


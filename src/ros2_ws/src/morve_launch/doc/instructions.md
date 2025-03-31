## Testing setup of the one robot wheel
1. on the Raspberry Pi start with use of the morve_launch package executable morve.launch.py it start's robot hardware interface and loads pid controller
2. start plotjuggler, ros2 run plotjuggler plotjuggler, and use introspection data values
3. the gains of the pid controller can be changed at runtime with use of
```
ros2 param set /pwm_pid_controller gains.front_left_wheel_joint.i 1.0

``` 
4. listening for changes on the controller can be monitored on topic /pwm_pid_controller/controller_state
```
ros2 topic echo /pwm_pid_controller/controller_state

```
5. basic command can be published to topic /pwm_pid_controller/reference
```
ros2 topic pub /pwm_pid_controller/reference control_msgs/msg/MultiDOFCommand "{dof_names: ['front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint'], values: [0.0, 0.0, 0.0, 0.0]}"
```

## Sending messages to diff drive controller
```
$ ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 2
```

## using teleop_twist_keyboard package for commanding to the diff drive controller
```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True -r /cmd_vel:=diff_drive_controller/cmd_vel
```


## listing available command interfaces
```
$ ros2 control list_hardware_interfaces
command interfaces
        front_left_wheel_joint/effort [available] [claimed]
        front_right_wheel_joint/effort [available] [claimed]
        pwm_pid_controller/front_left_wheel_joint/velocity [available] [unclaimed]
        pwm_pid_controller/front_right_wheel_joint/velocity [available] [unclaimed]
        pwm_pid_controller/rear_left_wheel_joint/velocity [available] [unclaimed]
        pwm_pid_controller/rear_right_wheel_joint/velocity [available] [unclaimed]
        rear_left_wheel_joint/effort [available] [claimed]
        rear_right_wheel_joint/effort [available] [claimed]
state interfaces
        front_left_wheel_joint/velocity
        front_right_wheel_joint/velocity
        pwm_pid_controller/front_left_wheel_joint/velocity
        pwm_pid_controller/front_right_wheel_joint/velocity
        pwm_pid_controller/rear_left_wheel_joint/velocity
        pwm_pid_controller/rear_right_wheel_joint/velocity
        rear_left_wheel_joint/velocity
        rear_right_wheel_joint/velocity
```

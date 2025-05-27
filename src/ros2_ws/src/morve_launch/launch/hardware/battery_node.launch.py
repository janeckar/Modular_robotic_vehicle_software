from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Used for changing the namespace of the battery_voltage node."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ina219_address",
            default_value="0x40",
            description="Address of the ina219 chip used for voltage measurements. Cannot be modified after start of the battery_voltage node with parameter.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "low_voltage_level",
            default_value="6.0",
            description="Low voltage level value when should the node send the message with low level as true if the actual voltage is less than this value."
        )
    )
    
    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    ina219_address = LaunchConfiguration("ina219_address")
    low_voltage_level = LaunchConfiguration("low_voltage_level")
    
    battery_node = Node(
        package="battery_voltage",
        executable="battery_voltage",
        output="both",
        parameters=[{"low_voltage_level": low_voltage_level, "ina219_address": ina219_address}],
        namespace=prefix
    )
    
    
    nodes = [
        battery_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
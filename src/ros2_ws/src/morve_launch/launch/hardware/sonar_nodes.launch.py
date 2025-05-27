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
            "sonar_parameters_file",
            default_value="sonar_params.yaml",
            description="Name of the sonar parameters file. This file should be located in the config/ subdirectory of the package specified by pkg_with_parameters parameter."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pkg_with_parameters",
            default_value="morve_launch",
            description="Package with parameters for the sonar nodes. This package should contain a file named by default \n \
                         sonar_params.yaml in config/ subdirectory."
        )
    )
    
    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    sonar_params_file_name = LaunchConfiguration("sonar_parameters_file")
    pkg_with_params = LaunchConfiguration("pkg_with_parameters")
    
    sonar_params_file_path = PathJoinSubstitution(
        [FindPackageShare(pkg_with_params), "config", sonar_params_file_name]
    )
    
    sonar_front_node = Node(
        package="sonar_distance",
        executable="sonar_node",
        name="sonar_front",
        output="both",
        parameters=[sonar_params_file_path],
        namespace=prefix
    )
    
    sonar_left_node = Node(
        package="sonar_distance",
        executable="sonar_node",
        name="sonar_left",
        output="both",
        parameters=[sonar_params_file_path],
        namespace=prefix
    )
    
    sonar_right_node = Node(
        package="sonar_distance",
        executable="sonar_node",
        name="sonar_right",
        output="both",
        parameters=[sonar_params_file_path],
        namespace=prefix
    )    
    
    nodes = [
        sonar_front_node,
        sonar_left_node,
        sonar_right_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
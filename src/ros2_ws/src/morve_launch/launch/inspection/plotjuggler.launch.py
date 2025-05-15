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
            description="Used for changing the namespace of the plotjuggler node."
        )
    )
    
    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        output="both",
        namespace=prefix
    )
    
    
    nodes = [
        plotjuggler_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
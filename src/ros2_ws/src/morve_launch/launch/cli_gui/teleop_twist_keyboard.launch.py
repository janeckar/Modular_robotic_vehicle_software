from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    terminal_name_arg = DeclareLaunchArgument(
        "terminal",
        default_value="konsole",
        description="Provide name of the terminal in which is the launch file started."
    )
            
    # initialize arguments
    terminal_name = LaunchConfiguration("terminal")
    
    teleop = ExecuteProcess(
        cmd=[[
            terminal_name, 
            ' -e sh -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard ',
            '--ros-args ',
            '-p stamped:=true"'
        ]],
        shell=True
    )

    return LaunchDescription([terminal_name_arg, teleop])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="morve_launch",
            description="Description package with robot URDF/xacro files. \
                This argument enables use of a custom description"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="morve.urdf.xacro",
            description="URDF/XACRO description file of the robot with ros2_control"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
                multi-robot setup. If changed than also joint names in controllers configuration \
                needs to be updated."
        )
    )
    
    # initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    
    # get URDF file with use of xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "description", description_file]
            ),
            " ",
            "prefix:=",
            prefix
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    nodes = [
        robot_state_pub_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
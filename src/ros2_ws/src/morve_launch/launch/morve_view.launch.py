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
            default_value="morve_description",
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
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint state publisher gui automatically.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
                multi-robot setup. If changed than also joint names in controllers configuration \
                needs to be updated."
        )
    )
    
    # initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    
    # get URDF file with use of xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "description/urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "description/rviz", "morve_view.rviz"]
    )
    
    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        namespace=prefix,
        condition=IfCondition(gui),
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace=prefix,
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    nodes = [
        joint_state_pub_node,
        robot_state_pub_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
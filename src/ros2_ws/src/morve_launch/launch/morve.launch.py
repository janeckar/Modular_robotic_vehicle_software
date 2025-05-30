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
            default_value="false",
            description="Start Rviz2 and Joint state publisher gui automatically.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup.\n\
        If changed then also joint names in controllers configuration needs to be updated."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="morve_dead_zone_controllers.yaml",
            description="Config file for setting up controllers.",
        )
    )
    
    # initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    controllers_file = LaunchConfiguration("controllers_file")
    
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
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("morve_launch"),
            "config",
            controllers_file,
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "description/rviz", "morve_view.rviz"]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        namespace=prefix,
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
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",],
    )
    
    PWM_dead_zone_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pwm_dead_zone_pid_controller",
            "diff_dead_zone_drive_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "--remap /diff_dead_zone_drive_controller/cmd_vel:=/cmd_vel",
        ],
    )
    
    PWM_pid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pwm_pid_controller",
            "diff_drive_controller",
            "--param-file",
            robot_controllers,
            "--load-only",
            "--controller-ros-args",
            "--remap /diff_drive_controller/cmd_vel:=/cmd_vel"
        ],
    )
    
    nodes = [
        control_node,
        robot_state_pub_node,
        PWM_dead_zone_pid_controller_spawner,
        PWM_pid_controller,
        joint_state_broadcaster_spawner,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
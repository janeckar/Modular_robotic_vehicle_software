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
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("morve_launch"),
            "config",
            "morve_controllers.yaml",
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
    
    PWM_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pwm_pid_controller",
            "diff_drive_controller",
            "--param-file",
            robot_controllers,
            #"--controller-ros-args",
            #"-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )
    
    # diff_drive_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "diff_drive_controller",
    #         "--param-file",
    #         robot_controllers,
    #         #"--controller-ros-args",
    #         #"-r diff"
    #     ]
    # )
    
    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )
    
    # delay_joint_state_broadcaster_after_PWM_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=PWM_pid_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )
    
    nodes = [
        control_node,
        robot_state_pub_node,
        PWM_pid_controller_spawner,
        # diff_drive_controller_spawner, 
        # delay_joint_state_broadcaster_after_PWM_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
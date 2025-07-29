# ✅ Standard ROS2 Pattern Launch File
# Based on urdf_launch/display.launch.py pattern with UR5e integration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )
    
    # Standard ROS2 pattern: jsp_gui argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'jsp_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        )
    )
    
    # RViz control argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable RViz'
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot description generation
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=", ur_type,
        " ",
        "use_fake_hardware:=true",  # For simulation
    ])
    
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Robot state publisher (always needed)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint state publisher (no GUI) - runs when jsp_gui=false
    # NOTE: Disabled to avoid conflict with programmatic control
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui)
    )

    # Joint state publisher GUI - runs when jsp_gui=true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui)
    )

    # RViz (optional)
    rviz_config_file = "/workspace/robot/rviz/my_phase1_config.rviz"
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    nodes_to_start = [
        robot_state_publisher_node,
        # joint_state_publisher_node,  # Disabled to avoid conflict
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
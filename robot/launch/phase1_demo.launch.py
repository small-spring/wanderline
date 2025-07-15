#!/usr/bin/env python3
"""
Phase 1 Demo Launch File

Launches Phase 1 robot drawing demo with existing robot simulation.
For VNC environment testing.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Phase 1 demo."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('robot') if os.path.exists('/opt/ros') else '/Users/smallspring/programs/wanderline/robot'
    
    launch_actions = []
    
    # 1. Include existing UR5e robot simulation
    try:
        ur5e_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'ur5e_standard.launch.py')
            )
        )
        launch_actions.append(ur5e_launch)
    except Exception as e:
        print(f"Warning: Could not include ur5e_standard.launch.py: {e}")
    
    # 2. Phase 1 main node
    phase1_node = Node(
        package='robot',
        executable='main.py',
        name='phase1_robot_drawer',
        output='screen',
        cwd=os.path.join(pkg_dir, 'demos', 'phase1'),
        parameters=[{
            'use_sim_time': True if os.path.exists('/opt/ros') else False
        }]
    )
    launch_actions.append(phase1_node)
    
    # 3. Optional: RViz for visualization
    rviz_config = os.path.join(pkg_dir, 'config', 'robot_visualization.rviz')
    if os.path.exists(rviz_config):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        launch_actions.append(rviz_node)
    
    return LaunchDescription(launch_actions)
# import os
# from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    test_config_file = PathJoinSubstitution([
        FindPackageShare('cvx_motion_planning'),
        'config',
        'parameters.yaml'
    ])

    moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lampo_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        )
    )

    test_solver = Node(
            package='cvx_motion_planning',
            executable='test_solver',
            name='test_solver_node',
            parameters=[test_config_file],
            remappings=[
                ('/test_solver_node/planning_scene','/planning_scene')
            ]
        )

    return LaunchDescription([
        test_solver,
        moveit_include
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    planner_config = os.path.join(
        get_package_share_directory('polytopes_centralized_planner'),
        'config',
        'parameters.yaml')

    planner_node =  Node(
            package='polytopes_centralized_planner',
            executable='centralized_planner_node',
            name='centralized_planner_node',
            output='screen',
            parameters=[planner_config,
                        {'use_sim_time' : True}],
            arguments=['--ros-args', '--log-level', "info"])

    nodes_to_start = [planner_node]

    return LaunchDescription(nodes_to_start)

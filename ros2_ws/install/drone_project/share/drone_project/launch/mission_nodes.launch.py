from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo 1: Interface
        Node(
            package='drone_project',
            executable='interface_node',  # Debe coincidir con setup.py
            name='interface_node',
            output='screen'
        ),
        
        # Nodo 2: Mission Control
        Node(
            package='drone_project',
            executable='mission_control_node', # Debe coincidir con setup.py
            name='mission_control_node',
            output='screen'
        ),
    ])

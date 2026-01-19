import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('topic_watchdog'),
        'config',
        'watchdog_params.yaml'
    )
    
    # Dynamically generate nodes out of yaml parameters
    with open(config_path, 'r') as f:
        config_params = yaml.safe_load(f)
        
    nodes_to_start = []
    
    for node_name, params in config_params.items():
        if node_name == '/**':
            continue
        
        new_node = Node(
            package='topic_watchdog',
            executable='watchdog_node',
            name=node_name,
            parameters=[config_path],
            remappings=[
                ('heartbeat',f'{node_name}/heartbeat'),
                ('heartbeat_debug',f'{node_name}/heartbeat_debug')
            ]
        )
        nodes_to_start.append(new_node)

    return LaunchDescription(nodes_to_start)
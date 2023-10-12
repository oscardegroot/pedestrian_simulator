import launch
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import yaml

def generate_launch_description():
    """Generate launch description with multiple components."""
    
    config_parameters = os.path.join(
        get_package_share_directory('pedestrian_simulator'),
        'config',
        'configuration.yaml'
    )
    
    config_dir = os.path.join(get_package_share_directory('pedestrian_simulator'), 'config')
    param_config = os.path.join(config_dir, 'configuration.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]

    node = ComposableNode(
                    package='pedestrian_simulator',
                    plugin='pedestrian_simulator::PedestrianSimulator',
                    name='pedestrian_simulator',
                    # remappings=[('/image', '/burgerimage')],
                    parameters=[params]                    
                    # extra_arguments=[{'use_intra_process_comms': True}]),
                )

    container = ComposableNodeContainer(
            name='pedestrian_simulator_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[node],
            output={'both':'screen'},
    )

    return launch.LaunchDescription([container])
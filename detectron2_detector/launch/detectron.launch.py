import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('detectron2_detector'),
        'config',
        'detectron2.yaml'
        )

    detectron_node = Node(
        package = 'detectron2_detector',
        name = 'detectron2_node',
        executable = 'detectron2_node',
        parameters = [config]
        )

    return launch.LaunchDescription([detectron_node])
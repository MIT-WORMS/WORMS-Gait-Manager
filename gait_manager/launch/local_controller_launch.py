import os
import pandas as pd

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    node = Node(package="gait_manager",
                executable='local_controller')
    return LaunchDescription([node])
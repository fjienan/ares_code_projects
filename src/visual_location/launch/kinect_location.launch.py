from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    kinect_driver_node = Node(
        package="azure_kinect_ros2_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True
    )
    ld.add_action(kinect_driver_node)

    params_file = "/home/ares/ares_code_projects/src/config/params.yaml"
    
    visual_location_node = Node(
        package='visual_location',
        executable='visual_location',
        name='visual_location_node',
        output='screen',
        parameters=[params_file]
    )
    ld.add_action(visual_location_node)
    
    return ld
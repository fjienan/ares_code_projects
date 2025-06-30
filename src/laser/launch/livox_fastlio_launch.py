#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    启动Livox驱动和Fast-LIO映射的launch文件
    """
    
    # 获取包的共享目录
    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    fastlio_share_dir = get_package_share_directory('fast_lio')
    
    # 启动Livox ROS驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )
    
    # 启动Fast-LIO映射
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_share_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'avia.yaml'
        }.items()
    )
    
    return LaunchDescription([
        livox_launch,
        fastlio_launch
    ]) 
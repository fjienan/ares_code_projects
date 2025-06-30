#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动初始位姿TF发布器节点的launch文件
    """
    
    # 创建initialpose_tf_publisher节点
    initialpose_tf_node = Node(
        package='laser',
        executable='initialpose_tf_publisher',
        name='initialpose_tf_publisher',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    return LaunchDescription([
        initialpose_tf_node
    ]) 
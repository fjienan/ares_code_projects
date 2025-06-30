#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations
import math

class InitialPoseTFPublisher(Node):
    def __init__(self):
        super().__init__('initialpose_tf_publisher')
        
        # 创建静态变换发布器
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 订阅初始位姿话题
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        self.get_logger().info("初始位姿TF发布器节点已启动")
        self.get_logger().info("等待来自rviz2的初始位姿设置...")

    def initialpose_callback(self, msg):
        """处理接收到的初始位姿消息"""
        try:
            # 创建静态变换消息
            static_transform = TransformStamped()
            
            # 设置变换头信息
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = 'map'
            static_transform.child_frame_id = 'camera_init'
            
            # 从PoseWithCovarianceStamped中提取位置和姿态信息
            pose = msg.pose.pose
            
            # 设置平移
            static_transform.transform.translation.x = pose.position.x
            static_transform.transform.translation.y = pose.position.y
            static_transform.transform.translation.z = pose.position.z
            
            # 设置旋转（四元数）
            static_transform.transform.rotation.x = pose.orientation.x
            static_transform.transform.rotation.y = pose.orientation.y
            static_transform.transform.rotation.z = pose.orientation.z
            static_transform.transform.rotation.w = pose.orientation.w
            
            # 发布静态变换
            self.tf_broadcaster.sendTransform(static_transform)
            
            # 转换四元数为欧拉角用于日志显示
            quaternion = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
            
            self.get_logger().info(
                f"发布map->camera_init静态变换:\n"
                f"  位置: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}\n"
                f"  姿态: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°"
            )
            
        except Exception as e:
            self.get_logger().error(f"处理初始位姿时发生错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
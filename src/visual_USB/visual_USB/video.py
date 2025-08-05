import numpy as np
try:
    from ultralytics import YOLO
except ImportError:
    print("Warning: ultralytics not available, YOLO functionality will not work")
    YOLO = None

try:
    from rclpy.node import Node
    import rclpy
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except ImportError:
    print("Warning: ROS2 not available, running in standalone mode")
    ROS_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped
except ImportError:
    print("Warning: ROS2 message types not available")
    CvBridge = None
    PoseStamped = None

try:
    import cv2
except ImportError:
    print("Warning: OpenCV not available")
    cv2 = None
from typing import Tuple

# Base class depends on whether ROS2 is available
if ROS_AVAILABLE:
    BaseClass = Node
else:
    BaseClass = object

class VisualPosition(BaseClass):
    """
    视觉定位节点，处理相机几何和目标检测
    """
    
    def __init__(self):
        if ROS_AVAILABLE:
            super().__init__('visual_position_node')
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('pitch', 0.0),
                    ('camera_height', 1.0),
                    ('hoop_height', 2430.0),
                    ('fx', 978.884521484375),
                    ('fy', 978.8125),
                    ('cx', 1019.759765625),
                    ('cy', 783.3111572265625),
                    ('camera_frequency', 0.1)
                ]
            )
            self.hoop_height = self.get_parameter('hoop_height').get_parameter_value().double_value
            self.fx = self.get_parameter('fx').get_parameter_value().double_value
            self.fy = self.get_parameter('fy').get_parameter_value().double_value
            self.cx = self.get_parameter('cx').get_parameter_value().double_value
            self.cy = self.get_parameter('cy').get_parameter_value().double_value
            self.frequency = self.get_parameter('camera_frequency').get_parameter_value().double_value
            self.pitch = self.get_parameter('pitch').get_parameter_value().double_value
            self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value

            self.get_logger().info(f"""
            hoop_height:{self.hoop_height}
            fx:{self.fx}
            fy:{self.fy}
            cx:{self.cx}
            cy:{self.cy}
            frequency:{self.frequency}
            pitch:{self.pitch}
            camera_height:{self.camera_height}
            """)
            
            # ROS2 specific initialization
            if CvBridge:
                self.bridge = CvBridge()
            if Image and PoseStamped:
                self.color_sub = self.create_subscription(Image, 'k4a/rgb/image_raw', self.image_callback, 10)
                self.pose_publisher = self.create_publisher(PoseStamped, 'visual_pose', 10)
                self.create_timer(self.frequency, self.timer_callback)
        else:
            # Non-ROS initialization with default values
            self.hoop_height = 2430.0
            self.fx = 978.884521484375
            self.fy = 978.8125
            self.cx = 1019.759765625
            self.cy = 783.3111572265625
            self.frequency = 0.1
            self.pitch = 0.0
            self.camera_height = 1.0
            
            print(f"""
            hoop_height:{self.hoop_height}
            fx:{self.fx}
            fy:{self.fy}
            cx:{self.cx}
            cy:{self.cy}
            frequency:{self.frequency}
            pitch:{self.pitch}
            camera_height:{self.camera_height}
            """)
            
        self.image = None
        # 初始化YOLO模型
        if YOLO:
            try:
                self.model = YOLO('/home/ares/Desktop/ultralytics/runs/detect/train15/weights/best.pt')
            except:
                self.model = None
        else:
            self.model = None

    def image_callback(self, msg):
        """
        图像回调函数
        """
        if ROS_AVAILABLE and CvBridge:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("--------------------------------Received image--------------------------------")
        else:
            print("Image callback called but ROS2/CvBridge not available")
    def detect_objects(self):
        """
        使用YOLO模型检测图像中的物体
        """
        if self.image is None:
            return None, None
                                                                                                                  
        x_center, y_center = 0, 0
        x1, y1, x2, y2 = 0, 0, 0, 0
        
        results = self.model(self.image, conf=0.25)[0]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
        """
        获取方形框的中心点坐标
        """
        if len(results.boxes) > 0:
            # 获取第一个检测到的物体
            box = results.boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            if ROS_AVAILABLE:
                self.get_logger().info(f"Center point: ({x_center}, {y_center})")
            else:
                print(f"Center point: ({x_center}, {y_center})")
            
            # 绘制中心点
            cv2.circle(self.image, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)
            cv2.rectangle(self.image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.imshow("image", self.image)
            cv2.waitKey(1)
            
            return x_center, y_center
        else:
            if ROS_AVAILABLE:
                self.get_logger().warn("No object detected")
            else:
                print("Warning: No object detected")
            if self.image is not None:
                cv2.imshow("image", self.image)
                cv2.waitKey(1)
            return None, None
    def locate_object(self, x_center, y_center):
        """
        基于单目相机几何的物体定位
        世界坐标系：x-左，y-上，z-前
        """
        # 将像素坐标转换为归一化相机坐标
        x_norm = (x_center - self.cx) / self.fx
        y_norm = (y_center - self.cy) / self.fy
        
        # 高度差（目标高度 - 相机高度）
        height_diff = self.hoop_height - self.camera_height
        
        # 考虑相机俯仰角，计算到目标的距离
        # pitch > 0 表示相机向下俯视
        denominator = -y_norm * np.cos(self.pitch) + np.sin(self.pitch)
        
        if abs(denominator) < 1e-6:
            if ROS_AVAILABLE:
                self.get_logger().warn("除零错误：相机视线与目标平面平行")
            else:
                print("Warning: 除零错误：相机视线与目标平面平行")
            return 10000, 10000, 10000
        
        # 计算到目标的距离（沿相机z轴方向）
        distance = height_diff / denominator
        
        if distance <= 0:
            if ROS_AVAILABLE:
                self.get_logger().warn("计算得到负距离，检查几何设置")
            else:
                print("Warning: 计算得到负距离，检查几何设置")
            return 10000, 10000, 10000
        
        # 世界坐标系中的位置
        x_world = -x_norm * distance  # 负号：像素x正方向对应世界x负方向
        z_world = distance * np.cos(self.pitch)  # 前方距离
        y_world = self.hoop_height  # 目标高度
        
        return x_world, y_world, z_world
        
    def timer_callback(self):
        """
        定时器回调函数
        """
        if self.image is not None:
            x_center, y_center = self.detect_objects()
            if x_center is not None and y_center is not None:
                # 计算世界坐标
                x, y, z = self.locate_object(x_center, y_center)
                if ROS_AVAILABLE:
                    self.get_logger().info(f"World position: {x}, {y}, {z}")
                else:
                    print(f"World position: {x}, {y}, {z}")
                
                # 发布位姿信息
                self.publish_pose(x, y, z)
        
    def publish_pose(self, x, y, z):
        """
        发布位姿信息
        """
        if not ROS_AVAILABLE or not PoseStamped:
            print(f"Would publish pose: x={float(x)/1000:.3f}, y={float(y)/1000:.3f}, z={float(z)/1000:.3f}")
            return
            
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_frame"
        
        # 设置位置 (世界坐标)
        pose.pose.position.x = float(x)/1000
        pose.pose.position.y = float(y)/1000
        pose.pose.position.z = float(z)/1000
        
        # 设置方向 (简单设置为单位四元数)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        if hasattr(self, 'pose_publisher'):
            self.pose_publisher.publish(pose)
            self.get_logger().info(f"Published pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")
        else:
            print(f"Would publish pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")

def main():
    if not ROS_AVAILABLE:
        print("Error: ROS2 is not available. Cannot start visual position node.")
        print("Please ensure ROS2 is properly installed and sourced.")
        return 1
    
    try:
        rclpy.init()
        visual_position = VisualPosition()
        rclpy.spin(visual_position)
        visual_position.destroy_node()
        rclpy.shutdown()
        return 0
    except Exception as e:
        print(f"Error running visual position node: {e}")
        return 1

if __name__ == '__main__':
    exit(main())
from math import atan2, cos, pi, sin, sqrt
import socket
import json
import time
import select
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from tf_transformations import euler_from_quaternion
import numpy as np


class GamepadData(Node):
    def __init__(self):
        # 初始化ROS节点
        super().__init__('robot_udp_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.position_publisher = self.create_publisher(Point, 'selected_position', 10)
        self.robot1_subscriber = self.create_subscription(PoseStamped, 'laser_position', self.robot1_callback, 10)
        self.robot2_subscriber = self.create_subscription(PoseStamped, 'robot2', self.robot2_callback, 10)
        self.laser_subscriber = self.create_subscription(Float32MultiArray, 'sensor_data', self.laser_callback, 10)
        self.get_logger().info("GamepadData Node has been started.")
        # UDP服务器配置
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 8888))
        self.UDP_IP = "192.168.31.22"
        self.UDP_PORT = 8889
        #sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.lock = threading.Lock()
        # 初始化变量
        self.latency_history = []
        self.latency_ms = 800
        self.gamepad = {'axes':[], 'buttons':[], 'latency':800, 'packetId':0}

        self.velocityX = 0.0
        self.velocityY = 0.0
        self.angularZ = 0.0
        self.lasers = [0,0,0,0,0]

        self.packet_id = 0
        self.record_packets = []
        self.valid_rate = 1.0
        threading.Thread(target=self.main, daemon=True).start()
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.timer2 = self.create_timer(0.01, self.send_position)

        self.robot1 = (100, 150, 0)  # 初始位置和角度
        self.robot2 = (100, 250, 0)  # 初始位置和角度
        self.target = (-1, -1)  # 初始目标位置
    def timer_callback(self):
        # 发布cmd_vel消息
        msg = Twist()
        msg.linear.x = self.velocityX
        msg.linear.y = self.velocityY
        msg.angular.z = self.angularZ
        # self.publisher_.publish(msg)
        # 发布目标位置 
        if self.target != (-1, -1):
            target_msg = Point() 
            target_msg.x = self.target[0] / 1050
            target_msg.y = self.target[1] / 560
            target_msg.z = 0.0
            self.target = (-1, -1)  # 重置目标位置
            self.position_publisher.publish(target_msg)
    def robot1_callback(self, msg):
        x = msg.pose.position.x*70
        y = msg.pose.position.y*70
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)
        self.robot1 = (x,y,yaw)
    def robot2_callback(self, msg):
        x = msg.pose.position.x*70
        y = msg.pose.position.y*70
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)
        self.robot2 = (x,y,yaw)

    def laser_callback(self, msg):
        laserInMeter = np.array(msg.data).tolist()
        for i in range(len(laserInMeter)):
            self.lasers[i] = laserInMeter[i]*70

    def send_position(self):
        position_data = {
            "type": "coordinate",
            "robot1": {
                "x": self.robot1[0],
                "y": self.robot1[1],
                "angle": self.robot1[2]
            },
            "robot2": {
                "x": self.robot2[0],
                "y": self.robot2[1],
                "angle": self.robot2[2]
            },
            "lasers": self.lasers
            
        }
        try:
            self.sock.sendto(json.dumps(position_data).encode(), (self.UDP_IP, self.UDP_PORT))
        except Exception as e:
            self.get_logger().error(f"Failed to send position data: {e}")
    def main(self):
        latency_ms = 0
        record_packets = []
        valid_rate = 1.0
        packet_id = 0
        self.latency_history = []
        gamepad = {'axes':[], 'buttons':[], 'latency':800, 'packetId':0}
        while True:
            # 接收数据（网页5的recvfrom实现
            readable, _, _ = select.select([self.sock], [], [], 0.01)
            if self.sock in readable:
                data, _ = self.sock.recvfrom(1024)
                gamepad = json.loads(data.decode())
            
                if gamepad["type"] == "ping":
                    reply = {
                        "type": "pong",
                        "timeId": gamepad["timeId"],
                        "valid": valid_rate
                    }
                    self.sock.sendto(json.dumps(reply).encode(), (self.UDP_IP, self.UDP_PORT))
                if gamepad["type"] != "coordinate":
                    latency_ms = gamepad["latency"]
                    packet_id = gamepad["packetId"]
                    if latency_ms < 100: 
                        record_packets.append((packet_id,time.time()))
                else:
                    self.target = (gamepad["x"]*1050, gamepad["y"]*560)
            self.latency_history.append(latency_ms)
            if len(self.latency_history) > 200:
                self.latency_history.pop(0)

            try:
                self.velocityX = -gamepad["axes"][1]
                self.velocityY = gamepad["axes"][0]
                self.angularZ = gamepad["axes"][3]
                if abs(self.velocityX) < 0.1:
                    self.velocityX = 0.0
                if abs(self.velocityY) < 0.1:
                    self.velocityY = 0.0
                if abs(self.angularZ) < 0.1:
                    self.angularZ = 0.0
            except Exception as e:
                pass  # 如果没有axes数据，忽略错误
                
            # 丢包率计算
            for i in range(0,len(record_packets)):
                if i >= len(record_packets) or i < 0 or len(record_packets) == 0:
                    break
                if record_packets[i][0] <= packet_id-200 or record_packets[i][1] <= time.time()-8:
                    record_packets.pop(i)
                    i -= 1
                if i >= len(record_packets) or i < 0 or len(record_packets) == 0:
                    break
                if record_packets[i][0] > packet_id:
                    record_packets = []
            
            valid_rate = len(record_packets)/200

            # 输出当前状态
            if time.time() % 1 < 0.02:
                self.get_logger().info(f"Average Latency: {((sum(self.latency_history)/len(self.latency_history)) if len(self.latency_history) > 0 else 0):.2f} ms, Packet ID: {packet_id}, Valid Rate: {valid_rate*100:.0f}%")
                try:
                    for i in range(len(gamepad["axes"])):
                        self.get_logger().info(f"Axis {i}: {gamepad['axes'][i]}")
                    for i in range(len(gamepad["buttons"])):
                        self.get_logger().info(f"Button {i}: "+ ("Pressed" if gamepad["buttons"][i] else "Released"))
                except Exception:
                    pass
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    gamepad_data = GamepadData()
    rclpy.spin(gamepad_data)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

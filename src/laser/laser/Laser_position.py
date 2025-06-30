import rclpy
from rclpy.node import Node
from geometry_msg.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import threading
from scipy.optimize import fsolve
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from laser.transformation import Transformation

class Laser_position(Node):
    def __init__(self):
        super().__init__('laser_position')
        self.get_logger().info('----start initializing the laser_position node----')
        self.subscription_laser=self.create_subscription(
            Float32MultiArray,
            '/sensor_data',
            self.laser_callback,
            10
        )#订阅laser/laser_data，队列长度为10，订阅的数据类型为Float32MultiArray，Float32MultiArray是自定义的数据类型

        self.create_subscription(
            Odometry,
            'Odometry',
            self.odo_callback,
            10
        )
        #订阅imu/odom，队列长度为10，订阅的数据类型为Odometry，Odometry是自定义的数据类型

        self.publisher_=self.create_publisher(PoseStamped,'/laser_position',10)#发布激光位置数据，发布到/laser_position，队列长度为10，发布的数据类型为LaserPosition，LaserPosition是自定义的数据类型

        self.declare_parameters(
            namespace='',
            parameters=[
                ('FIELD_SIZE', [0.00,0.00]),
                ('three_D_noise_std', [0.00,0.00,0.00]),
                ('measurement_noise_std', [0.00,0.00,0.00]),
                ('START_POINT', [0.00,0.00]),
                ('LASER_ALLOWED_NOISE', 0.00),
                ('FREQUENCY', 0.00),
                ('DELTA_DISTANCE', 0.00),
                ('three_D_DISTANCE',0.00),
                ('GRAVITY', 0.00),
                ('angle_bios',0.00)
            ]
        )
        self.FIELD_SIZE = self.get_parameter('FIELD_SIZE').get_parameter_value().double_array_value
        self.three_D_noise_std = self.get_parameter('three_D_noise_std').get_parameter_value().double_array_value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').get_parameter_value().double_array_value
        self.START_POINT = self.get_parameter('START_POINT').get_parameter_value().double_array_value
        self.LASER_ALLOWED_NOISE = self.get_parameter('LASER_ALLOWED_NOISE').get_parameter_value().double_value
        self.FREQUENCY = self.get_parameter('FREQUENCY').get_parameter_value().double_value
        self.DELTA_DISTANCE = self.get_parameter('DELTA_DISTANCE').get_parameter_value().double_value
        self.three_D_DISTANCE = self.get_parameter('three_D_DISTANCE').get_parameter_value().double_value
        self.GRAVITY = self.get_parameter('GRAVITY').get_parameter_value().double_value
        self.angle_bios = self.get_parameter('angle_bios').get_parameter_value().double_value
        self.get_logger().info(f"""
        Loaded Parameters:
        - FIELD_SIZE = {self.FIELD_SIZE}
        - three_D_noise_std = {self.three_D_noise_std}
        - measurement_noise_std = {self.measurement_noise_std}
        - START_POINT = {self.START_POINT}
        - LASER_ALLOWED_NOISE = {self.LASER_ALLOWED_NOISE}
        - FREQUENCY = {self.FREQUENCY}
        - DELTA_DISTANCE = {self.DELTA_DISTANCE}
        - three_D_DISTANCE = {self.three_D_DISTANCE}
        - GRAVITY = {self.GRAVITY}
        - angle_bios = {self.angle_bios}
        """) ##确认参数是否正确加载

        self.yaw = None 
        self.odo_position = None
        self.laser_data=[-1] * len(LASER_ANGLES)
        self.angle_1 = None
        self.angle_2 = None
        self.angle_3 = None
        self.angle_4 = None
        self.side_1_laser_data = []
        self.side_2_laser_data = []
        self.side_3_laser_data = []
        self.side_4_laser_data = []
        self.laser_angel_list = [np.deg2rad(0)+self.angle_bios,np.deg2rad(72)+self.angle_bios,np.deg2rad(144)+self.angle_bios,np.deg2rad(216)+self.angle_bios,np.deg2rad(288)+self.angle_bios]
        self.position = None
        self.laser_position_flag = False

        self.timer=self.create_timer(self.FREQUENCY,self.timer_callback)
    def laser_callback(self,msg):
        self.laser_data = np.array(msg.data)
        if self.laser_data != [-1] * len(self.laser_data):
            self.laser_position_flag = True
        else:
            self.laser_position_flag = False
    def odo_callback(self, msg):
        pose = [msg.pose.pose.position.y,msg.pose.pose.position.x]
        self.odo_quaternion=[
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        #转换成欧拉角
        self.yaw = euler_from_quaternion(self.odo_quaternion)[2]

    def criteria(self,estimate_position):
        self.angle_1 = np.arctan((self.FIELD_SIZE[1]- estimate_position[1]) / (self.FIELD_SIZE[0]-estimate_position[0])) 
        self.angle_2 = np.pi/2 + np.arctan((estimate_position[0] / self.FIELD_SIZE[1] - estimate_position[1]))  
        self.angle_3 = np.pi + np.arctan((estimate_position[1] / estimate_position[1]))
        self.angle_4 = np.pi*3/2 + np.arctan((self.FIELD_SIZE[0] - estimate_position[0]) / estimate_position[1])
    def classify_laser_data(self,laser_data):
        self.criteria(self.odo_position)
        for i in range(len(laser_data)):
            if 0 <= self.laser_angel_list[i] < self.angle_1 or self.angle_4 <= self.laser_angel_list[i] < np.pi*2:
                self.side_1_laser_data.append([laser_data[i],i])
            elif self.angle_1 <= self.laser_angel_list[i] < self.angle_2:
                self.side_2_laser_data.append([laser_data[i],i])
            elif self.angle_2 <= self.laser_angel_list[i] < self.angle_3:
                self.side_3_laser_data.append([laser_data[i],i])
            elif self.angle_3 <= self.laser_angel_list[i] < self.angle_4:
                self.side_4_laser_data.append([laser_data[i],i])

    def calculate_laser_position(self):
        self.classify_laser_data(self.laser_data)
        list_1 = []
        list_2 = []
        list_3 = []
        list_4 = []
        def equation(angle):
            for dis,id in self.side_1_laser_data:
                list_1.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
            for dis,id in self.side_2_laser_data:
                list_2.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
            for dis,id in self.side_3_laser_data:
                list_3.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
            for dis,id in self.side_4_laser_data:
                list_4.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
            lenth_1 = np.mean(list_1)
            lenth_2 = np.mean(list_2)
            lenth_3 = np.mean(list_3)
            lenth_4 = np.mean(list_4)
            return self.FIELD_SIZE[1] - lenth_2 - lenth_4
        angle = fsolve(equation,self.yaw)[0]
        for dis,id in self.side_3_laser_data:
            list_3.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
        for dis,id in self.side_4_laser_data:
            list_4.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
        x = np.mean(list_3)
        y = np.mean(list_4)
        self.get_logger().info(f"laser_position: {x}, {y},angle:{angle}")
        return [x,y,angle]
    def information_combination(self):
        self.position = self.calculate_laser_position()
        def calculate_noise(three_D_noise,measurement_noise):
            coefficient = []
            for i in range(3):
                coefficient.append([three_D_noise[i]/(three_D_noise[i] + measurement_noise[i]),measurement_noise[i]/(three_D_noise[i] + measurement_noise[i])])
            return coefficient
        coefficient = calculate_noise(self.three_D_noise_std,self.measurement_noise_std)
        if self.laser_position_flag:
            self.laser_position_flag = False
            self.position[0] = self.odo_position[0][0] * coefficient[0][0] + self.position[0] * coefficient[0][1]
            self.position[1] = self.odo_position[1][0] * coefficient[1][0] + self.position[1] * coefficient[1][1]
            self.position[2] = self.yaw * coefficient[2][0] + self.position[2] * coefficient[2][1]
        else:
            self.laser_position_flag = False
            self.position[0] = self.odo_position[0][0] 
            self.position[1] = self.odo_position[1][0] 
            self.position[2] = self.yaw 
        return self.position

    def timer_callback(self):
        state = self.information_combination()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = state[0]
        msg.pose.position.y = state[1]
        msg.pose.position.z = 0.00
        roll = 0.0
        pitch = 0.0
        yaw = float(state[2])
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    laser_position = Laser_position()
    rclpy.spin(laser_position)
    laser_position.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
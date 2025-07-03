import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from scipy.optimize import fsolve, root
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from laser.transformation import Transformation

class Laser_position(Node):
    def __init__(self):
        super().__init__('laser_position_node')
        self.get_logger().info('----start initializing the laser_position node----')
        self.START_POINT = []
        self.START_ANGLE = 0.0
        self.tag = False
        self.declare_parameters(                   
            namespace='',
            parameters=[
                ('world_size', [15.00, 8.00]),
                ('three_D_noise_std', [0.035, 0.07, 0.05]),
                ('measurement_noise_std', [0.01, 0.5, 0.1]),
                ('START_POINT', [6.068, 3.57]),
                ('LASER_ALLOWED_NOISE', 1.0),
                ('frequency', 0.2),
                ('DELTA_DISTANCE', 0.1247),
                ('angle_bios',0.0),
                ('three_D_position',[0.0,0.0]),
                ('three_d_angle',0.0)
            ]   
        )
        self.FIELD_SIZE = self.get_parameter('world_size').get_parameter_value().double_array_value
        self.three_D_noise_std = self.get_parameter('three_D_noise_std').get_parameter_value().double_array_value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').get_parameter_value().double_array_value
        self.START_POINT_1 = self.get_parameter('START_POINT').get_parameter_value().double_array_value
        self.LASER_ALLOWED_NOISE = self.get_parameter('LASER_ALLOWED_NOISE').get_parameter_value().double_value
        self.FREQUENCY = self.get_parameter('frequency').get_parameter_value().double_value
        self.DELTA_DISTANCE = self.get_parameter('DELTA_DISTANCE').get_parameter_value().double_value
        self.angle_bios = self.get_parameter('angle_bios').get_parameter_value().double_value
        self.three_D_position = self.get_parameter('three_D_position').get_parameter_value().double_array_value
        self.three_d_angle = self.get_parameter('three_d_angle').get_parameter_value().double_value
        
        self.get_logger().info(f"""
        Loaded Parameters:
        - FIELD_SIZE = {self.FIELD_SIZE}
        - three_D_noise_std = {self.three_D_noise_std}
        - measurement_noise_std = {self.measurement_noise_std}
        - LASER_ALLOWED_NOISE = {self.LASER_ALLOWED_NOISE}
        - FREQUENCY = {self.FREQUENCY}
        - DELTA_DISTANCE = {self.DELTA_DISTANCE}
        - angle_bios = {self.angle_bios}
        - START_POINT = {self.START_POINT_1}
        - three_D_position = {self.three_D_position}
        - three_d_angle = {self.three_d_angle}
        """) ##确认参数是否正确加载
        self.laser_angel_list = [np.deg2rad(0)+np.deg2rad(self.angle_bios),np.deg2rad(72)+np.deg2rad(self.angle_bios),np.deg2rad(144)+np.deg2rad(self.angle_bios),np.deg2rad(216)+np.deg2rad(self.angle_bios),np.deg2rad(288)+np.deg2rad(self.angle_bios)]
        # self.yaw = np.deg2rad(160.68267)
        # self.odo_position = np.array([7.93983,3.52614]).reshape(2,1)
        # self.laser_data = [8.4135,5.62544,5.44056,7.3704,3.52707]
        self.yaw = 0.0
        self.laser_data = [-1] * len(self.laser_angel_list)
        self.odo_position = []
        self.angle_1 = None
        self.angle_2 = None
        self.angle_3 = None
        self.angle_4 = None
        self.side_1_laser_data = []
        self.side_2_laser_data = []
        self.side_3_laser_data = []
        self.side_4_laser_data = []
        self.position = None
        self.laser_position_flag = True
        
        self.create_subscription(
            Float32MultiArray,
            '/sensor_data',
            self.laser_callback,
            10
        )#订阅laser/laser_data，队列长度为10，订阅的数据类型为Float32MultiArray，Float32MultiArray是自定义的数据类型

        self.create_subscription(
            Odometry,
            '/Odometry',
            self.odo_callback,
            10
        )
        #订阅imu/odom，队列长度为10，订阅的数据类型为Odometry，Odometry是自定义的数据类型
        self.publisher_=self.create_publisher(PoseStamped,'/laser_position',10)#发布激光位置数据，发布到/laser_position，队列长度为10，发布的数据类型为LaserPosition，LaserPosition是自定义的数据类型
        
        self.timer=self.create_timer(float(self.FREQUENCY),self.timer_callback)
        self.transformation = Transformation(np.deg2rad(self.three_d_angle), True, 2)
    
    def laser_callback(self,msg):
        self.laser_data = np.array(msg.data)
        for i in range(len(self.laser_data)):
            self.laser_data[i] = self.laser_data[i] + self.DELTA_DISTANCE
        if not self.tag:
            self.criteria(self.START_POINT_1)
            self.classify_laser_data(self.laser_data)
            self.START_POINT = self.calculate_laser_position()
            self.START_ANGLE = self.START_POINT[2]
            self.tag = True
            self.get_logger().info(f"---START_POINT: {self.START_POINT}")
         
    def odo_callback(self, msg):
        # self.odo_position = np.array([msg.pose.pose.position.y,msg.pose.pose.position.x]).reshape(2,1)
        pose = []
        pose = self.transformation.transformation(np.array([msg.pose.pose.position.x,msg.pose.pose.position.y]).reshape(2,1),2)
        self.odo_position = np.array([self.START_POINT[0] + pose[0] ,self.START_POINT[1] + pose[1] ]).reshape(2,1)
        self.odo_quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.yaw = euler_from_quaternion(self.odo_quaternion)[2] + self.START_ANGLE
        self.get_logger().info(f"position: {self.odo_position}")
        self.get_logger().info(f"yaw: {self.yaw}")

    def criteria(self,estimate_position):
        self.angle_1 = np.arctan((self.FIELD_SIZE[1]- estimate_position[1]) / (self.FIELD_SIZE[0]-estimate_position[0])) 
        self.angle_2 = np.pi/2 + np.arctan(estimate_position[0] / (self.FIELD_SIZE[1] - estimate_position[1]))  
        self.angle_3 = np.pi + np.arctan((estimate_position[1] / estimate_position[0]))
        self.angle_4 = np.pi*3/2 + np.arctan((self.FIELD_SIZE[0] - estimate_position[0]) / estimate_position[1])
        self.get_logger().info(f"angle_1: {np.rad2deg(self.angle_1)}, angle_2: {np.rad2deg(self.angle_2)}, angle_3: {np.rad2deg(self.angle_3)}, angle_4: {np.rad2deg(self.angle_4)}")

    def classify_laser_data(self,laser_data):
        # 清空前一次计算的数据
        self.side_1_laser_data = []
        self.side_2_laser_data = []
        self.side_3_laser_data = []
        self.side_4_laser_data = []
        
        
        for i in range(len(laser_data)):
            if 0 <= (self.laser_angel_list[i] + self.yaw)%(np.pi*2) < self.angle_1 or self.angle_4 <= (self.laser_angel_list[i] + self.yaw)%(np.pi*2)< np.pi*2:
                self.side_1_laser_data.append([laser_data[i],i])
            elif self.angle_1 <= (self.laser_angel_list[i] + self.yaw)%(np.pi*2) < self.angle_2:
                self.side_2_laser_data.append([laser_data[i],i])
            elif self.angle_2 <= (self.laser_angel_list[i] + self.yaw)%(np.pi*2) < self.angle_3:
                self.side_3_laser_data.append([laser_data[i],i])
            elif self.angle_3 <= (self.laser_angel_list[i] + self.yaw)%(np.pi*2) < self.angle_4:
                self.side_4_laser_data.append([laser_data[i],i])

    def calculate_laser_position(self):
        self.classify_laser_data(self.laser_data)
        
        # 添加调试信息
        self.get_logger().info(f"side_1_data: {self.side_1_laser_data}")
        self.get_logger().info(f"side_2_data: {self.side_2_laser_data}")
        self.get_logger().info(f"side_3_data: {self.side_3_laser_data}")
        self.get_logger().info(f"side_4_data: {self.side_4_laser_data}")

        # 放弃让计算机解方程，选择人手求出解析解
        a = 0
        b = 0
        c = self.FIELD_SIZE[1]
        for dis,id in self.side_2_laser_data:
            a += dis*np.cos(self.laser_angel_list[id])/len(self.side_2_laser_data)
            b += dis*np.sin(self.laser_angel_list[id])/len(self.side_2_laser_data)
        for dis,id in self.side_4_laser_data:
            a -= dis*np.cos(self.laser_angel_list[id])/len(self.side_4_laser_data)
            b -= dis*np.sin(self.laser_angel_list[id])/len(self.side_4_laser_data)
        # asin(x) + bcos(x) = c
        angle = np.arcsin(c/np.sqrt(a**2+b**2)) - np.arctan(b/a)
        self.get_logger().info(f"angle: {angle}")
        # 用解出的角度重新计算位置

        list_1 = []
        list_2 = []
        list_3 = []
        list_4 = []
        for dis,id in self.side_1_laser_data:
            list_1.append(abs(dis*np.cos(self.laser_angel_list[id]+angle)))
        for dis,id in self.side_2_laser_data:
            list_2.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
        for dis,id in self.side_3_laser_data:
            list_3.append(abs(dis*np.cos(self.laser_angel_list[id]+angle)))
        for dis,id in self.side_4_laser_data:
            list_4.append(abs(dis*np.sin(self.laser_angel_list[id]+angle)))
        
        # 处理空数组情况
        if len(list_3) == 0:
            x = self.FIELD_SIZE[0] - np.mean(list_1)
        elif len(list_3) == 0 and len(list_1) == 0:
            x = self.odo_position[0][0]
            self.get_logger().info(f"at this place, we cann't get the x position")
        elif len(list_1) == 0:
            x = np.mean(list_3)
        else:
            x = (np.mean(list_3) + self.FIELD_SIZE[0]-np.mean(list_1))/2

        if len(list_4) == 0:
            y = self.FIELD_SIZE[1] - np.mean(list_2)
        elif len(list_2) == 0:
            y = np.mean(list_4)
        else:
            y = (np.mean(list_4) + self.FIELD_SIZE[1]-np.mean(list_2))/2

        
        self.get_logger().info(f"laser_data: {self.laser_data}, yaw:{self.yaw}, odo_position:{self.odo_position}")
        self.get_logger().info(f"laser_position__: {x}, {y}, angle:{angle}")
        return [x, y, angle]

    def information_combination(self):
        self.position = self.calculate_laser_position()
        def calculate_noise(three_D_noise,measurement_noise):
            coefficient = []
            for i in range(3):
                coefficient.append([measurement_noise[i]/(three_D_noise[i] + measurement_noise[i]), three_D_noise[i]/(three_D_noise[i] + measurement_noise[i])])
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
        self.criteria(self.odo_position)
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
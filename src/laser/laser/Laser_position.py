import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from scipy.optimize import fsolve, root
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from transformation import Transformation

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
                ('three_d_angle',0.0),
                ('laser_angle_distribution',[0.0,72.0,144.0,216.0,288.0])
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
        self.laser_angle_list = self.get_parameter('laser_angle_distribution').get_parameter_value().double_array_value
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
        - laser_angle_distribution = {self.laser_angle_list}
        """) ##确认参数是否正确加载
        for i in range(len(self.laser_angle_list)):
            self.laser_angle_list[i] = np.deg2rad(self.laser_angle_list[i]) + np.deg2rad(self.angle_bios)
        # self.yaw = np.deg2rad(318.03751)
        # self.odo_position = np.array([9.3965,8-3.93917]).reshape(2,1)
        # self.laser_data = [6.07323, 6.47281, 4.02774, 9.44761 ,4.44383]
        self.yaw = 0.0
        self.laser_data = [-1] * len(self.laser_angle_list)
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
        self.laser_data_flag = [0] * len(self.laser_angle_list)
        
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
            if self.laser_data[i] != 0:
                self.laser_data_flag[i] = 1
            else:
                self.laser_data_flag[i] = 0
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
        # self.get_logger().info(f"position: {self.odo_position}")
        # self.get_logger().info(f"yaw: {self.yaw}")

    def criteria(self,estimate_position):
        self.angle_1 = np.arctan((self.FIELD_SIZE[1]- estimate_position[1]) / (self.FIELD_SIZE[0]-estimate_position[0])) 
        self.angle_2 = np.pi/2 + np.arctan(estimate_position[0] / (self.FIELD_SIZE[1] - estimate_position[1]))  
        self.angle_3 = np.pi + np.arctan((estimate_position[1] / estimate_position[0]))
        self.angle_4 = np.pi*3/2 + np.arctan((self.FIELD_SIZE[0] - estimate_position[0]) / estimate_position[1])

    def theory_length_of_laser(self, position, angle):
        distances = []
        
        for i, laser_angle in enumerate(self.laser_angle_list):
            # 计算激光方向 (相对于机器人朝向)
            absolute_angle = angle + laser_angle
            dx = math.cos(absolute_angle)
            dy = math.sin(absolute_angle)
            
            # 从机器人位置发射激光，检查与场地边界的交点
            min_distance = 20
            
            # 检查与场地边界的交点
            boundaries = [
                [(0, 0), (self.FIELD_SIZE[0], 0)],  # 下边界
                [(self.FIELD_SIZE[0], 0), (self.FIELD_SIZE[0], self.FIELD_SIZE[1])],  # 右边界
                [(self.FIELD_SIZE[0], self.FIELD_SIZE[1]), (0, self.FIELD_SIZE[1])],  # 上边界
                [(0, self.FIELD_SIZE[1]), (0, 0)],  # 左边界
            ]
            
            for boundary in boundaries:
                start, end = boundary
                intersection_dist = self.line_intersection_distance(
                    position[0], position[1], dx, dy,
                    start[0], start[1], end[0], end[1]
                )
                if intersection_dist is not None and intersection_dist < min_distance:
                    min_distance = intersection_dist
            
            # 添加距离偏置
            distances.append(max(0.1, min_distance))
            
        return distances

    def classify_laser_data(self,laser_data):
        theory_laser_data = []
        theory_laser_data = self.theory_length_of_laser(self.odo_position,self.yaw)
        # 清空前一次计算的数据
        self.side_1_laser_data = []
        self.side_2_laser_data = []
        self.side_3_laser_data = []
        self.side_4_laser_data = []
        
        for i in range(len(laser_data)):
            if laser_data[i] - theory_laser_data[i] > self.LASER_ALLOWED_NOISE:
                self.get_logger().warn(f"NUM {i + 1} laser data is not correct")
                self.laser_data_flag[i] = 0
                continue
            else:
                if 0 <= (self.laser_angle_list[i] + self.yaw)%(np.pi*2) < self.angle_1 or self.angle_4 <= (self.laser_angle_list[i] + self.yaw)%(np.pi*2)< np.pi*2:
                    self.side_1_laser_data.append([laser_data[i],i])
                elif self.angle_1 <= (self.laser_angle_list[i] + self.yaw)%(np.pi*2) < self.angle_2:
                    self.side_2_laser_data.append([laser_data[i],i])
                elif self.angle_2 <= (self.laser_angle_list[i] + self.yaw)%(np.pi*2) < self.angle_3:
                    self.side_3_laser_data.append([laser_data[i],i])
                elif self.angle_3 <= (self.laser_angle_list[i] + self.yaw)%(np.pi*2) < self.angle_4:
                    self.side_4_laser_data.append([laser_data[i],i])
        self.get_logger().info(f"valid laser data: {self.laser_data_flag}")
    def calculate_laser_position(self):
        self.classify_laser_data(self.laser_data)
        
        # 添加调试信息
        self.get_logger().info(f"side_1_data: {self.side_1_laser_data}")
        self.get_logger().info(f"side_2_data: {self.side_2_laser_data}")
        self.get_logger().info(f"side_3_data: {self.side_3_laser_data}")
        self.get_logger().info(f"side_4_data: {self.side_4_laser_data}")

        if len(self.side_2_laser_data) != 0  and len(self.side_4_laser_data) != 0:
            # 使用解析解计算角度
            self.a = 0
            self.b = 0
            # 从上墙和下墙的激光数据计算参数
            for dis, id in self.side_2_laser_data:
                self.a += dis * np.cos(self.laser_angle_list[id]) / len(self.side_2_laser_data)
                self.b += dis * np.sin(self.laser_angle_list[id]) / len(self.side_2_laser_data)
            for dis, id in self.side_4_laser_data:
                self.a -= dis * np.cos(self.laser_angle_list[id]) / len(self.side_4_laser_data)
                self.b -= dis * np.sin(self.laser_angle_list[id]) / len(self.side_4_laser_data)
            # asin(x)+bcos(x)=c
            block = self.c/np.sqrt(self.a**2+self.b**2)
            print("block",block)
            angle = 0
            if block > 1:
                block = 1
            if self.a < 0 and self.b != 0:
                phi = np.arctan(self.b/self.a) + np.pi
                print("--Step1--")
            elif self.a > 0 and self.b != 0:
                phi = np.arctan(self.b/self.a) 
                print("--Step2--")
            elif abs(self.a) < 0.00001 and self.b != 0:
                print("--Step3--")
                angle = np.arccos(self.c/self.b)
            elif abs(self.b) < 0.00001 and self.a != 0:
                print("--Step4--")
                angle = np.arcsin(self.c/self.a)
            print("angle",angle)
            if not angle:
                print("step_1")
                angle_1 = np.arcsin(block) - phi
                angle_1 = angle_1 % (np.pi*2)
                angle_2 = np.pi - angle_1 - 2*phi
                angle_2 = angle_2 % (np.pi*2)
                print("1--",angle_1-self.yaw)
                print("2--",angle_2-self.yaw)
                delta_1 = 0
                delta_2 = 0
                if angle_1 - self.yaw > np.pi:
                    delta_1 = 2*np.pi - abs(angle_1-self.yaw)   
                else:
                    delta_1 = abs(angle_1-self.yaw)
                if angle_2 - self.yaw > np.pi:
                    delta_2 = 2*np.pi - abs(angle_2-self.yaw)
                else:
                    delta_2 = abs(angle_2-self.yaw)
                angle = angle_2 if delta_1 > delta_2 else angle_1
                print("angle",angle)
            angle = angle % (np.pi*2)
            list_1 = []
            list_2 = []
            list_3 = []
            list_4 = []
            for dis,id in self.side_1_laser_data:
                list_1.append(abs(dis*np.cos(self.laser_angle_list[id]+angle)))
            for dis,id in self.side_2_laser_data:
                list_2.append(abs(dis*np.sin(self.laser_angle_list[id]+angle)))
            for dis,id in self.side_3_laser_data:
                list_3.append(abs(dis*np.cos(self.laser_angle_list[id]+angle)))
            for dis,id in self.side_4_laser_data:
                list_4.append(abs(dis*np.sin(self.laser_angle_list[id]+angle)))
            # 处理空数组情况
            if len(list_3) == 0:
                x = self.FIELD_SIZE[0] - np.mean(list_1)
            elif len(list_3) == 0 and len(list_1) == 0:
                x = self.odo_position[0][0]
                self.get_logger().warn(f"at this place, we cann't get the x position")
            elif len(list_1) == 0:
                x = np.mean(list_3)
            else:
                x = (np.mean(list_3) + self.FIELD_SIZE[0]-np.mean(list_1))/2
            y = (np.mean(list_4) + self.FIELD_SIZE[1]-np.mean(list_2))/2
            self.get_logger().info(f"laser_data: {self.laser_data}, yaw:{self.yaw}, odo_position:{self.odo_position}")
            self.get_logger().info(f"laser_position__: {x}, {y}, angle:{angle}")
            return [x, y, angle]
        else:
            self.get_logger().warn(f"!!!In this place, laser data is not valid!!!")
            angle = self.yaw
            x = self.odo_position[0][0]
            y = self.odo_position[1][0]
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
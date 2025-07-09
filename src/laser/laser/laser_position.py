import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from scipy.optimize import fsolve
import numpy as np
import math
from typing import List, Tuple, Optional

class RobotLaserSimulator:
    def __init__(self):
        # 场地参数 (与ROS节点保持一致)
        #language
        
        self.field_width: float = 15.0  # 15米
        self.field_height: float = 8.0   # 8米
        self.FIELD_SIZE = [self.field_width, self.field_height]
        
        # 机器人参数
        self.robot_x: float = 7.5  # 机器人初始位置X (场地中心)
        self.robot_y: float = 4.0  # 机器人初始位置Y (场地中心)
        self.robot_angle: float = 0.0  # 机器人初始角度 (弧度)
        self.robot_size: float = 0.5   # 机器人大小 (米)
        
        # 激光雷达参数 (与ROS节点保持一致)
        self.laser_count: int = 5
        self.laser_max_range: float = 20.0  # 最大探测距离10米
        self.angle_bios: float = 0.0  # 角度偏置
        # 5个激光器的角度：0°, 72°, 144°, 216°, 288° 加上偏置
        self.laser_angel_list = [
            np.deg2rad(0) + np.deg2rad(self.angle_bios),
            np.deg2rad(72) + np.deg2rad(self.angle_bios),
            np.deg2rad(144) + np.deg2rad(self.angle_bios),
            np.deg2rad(216) + np.deg2rad(self.angle_bios),
            np.deg2rad(288) + np.deg2rad(self.angle_bios)
        ]
        
        # 定位算法相关参数
        self.DELTA_DISTANCE: float = 0  # 距离偏置
        self.yaw: float = 0.0  # 机器人真实朝向角
        self.laser_data: List[float] = [0.0] * 5  # 激光测距数据
        
        # 角度阈值 (用于分类激光数据)
        self.angle_1: Optional[float] = None
        self.angle_2: Optional[float] = None  
        self.angle_3: Optional[float] = None
        self.angle_4: Optional[float] = None
        
        # 分类后的激光数据
        self.side_1_laser_data: List[Tuple[float, int]] = []
        self.side_2_laser_data: List[Tuple[float, int]] = []
        self.side_3_laser_data: List[Tuple[float, int]] = []
        self.side_4_laser_data: List[Tuple[float, int]] = []
        
        # 定位结果
        self.calculated_position: Optional[List[float]] = None
        self.position_error: Optional[float] = None
        
        # 交互参数
        self.dragging: bool = False
        self.rotating: bool = False
        self.last_mouse_pos: Optional[Tuple[float, float]] = None
        
        # matplotlib 组件类型注解
        self.fig: Figure
        self.ax: Axes
        self.c = self.FIELD_SIZE[1]
        self.a = 0
        self.b = 0
        # 初始化matplotlib
        self.setup_plot()
        
    def setup_plot(self):
        """设置matplotlib图形"""
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_xlim(0, self.field_width)
        self.ax.set_ylim(0, self.field_height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Robot Laser Positioning Simulation - Left: move, Right: rotate, A/D(±5°), Q/E(±1°)')
        
        # Bind mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # 初始化绘图元素
        self.robot_patch = None
        self.laser_lines = []
        self.info_text = None
        
        self.update_plot()
        
    def criteria(self, estimate_position: List[float]):
        """计算角度阈值，用于分类激光数据到不同墙面"""
        self.angle_1 = np.arctan((self.FIELD_SIZE[1] - estimate_position[1]) / (self.FIELD_SIZE[0] - estimate_position[0])) 
        self.angle_2 = np.pi/2 + np.arctan(estimate_position[0] / (self.FIELD_SIZE[1] - estimate_position[1]))  
        self.angle_3 = np.pi + np.arctan((estimate_position[1] / estimate_position[0]))
        self.angle_4 = np.pi*3/2 + np.arctan((self.FIELD_SIZE[0] - estimate_position[0]) / estimate_position[1])
        
    def classify_laser_data(self, laser_data: List[float]):
        """根据激光角度和机器人朝向，将激光数据分类到不同的墙面"""
        # 清空前一次计算的数据
        self.side_1_laser_data = []
        self.side_2_laser_data = []
        self.side_3_laser_data = []
        self.side_4_laser_data = []
        
        for i in range(len(laser_data)):
            # 计算激光的绝对角度 (激光相对角度 + 机器人朝向角)
            absolute_angle = (self.laser_angel_list[i] + self.yaw) % (np.pi * 2)
            
            # 根据角度阈值分类到不同墙面
            if (0 <= absolute_angle < self.angle_1) or (self.angle_4 <= absolute_angle < np.pi*2):
                self.side_1_laser_data.append([laser_data[i], i])  # 右墙
            elif self.angle_1 <= absolute_angle < self.angle_2:
                self.side_2_laser_data.append([laser_data[i], i])  # 上墙
            elif self.angle_2 <= absolute_angle < self.angle_3:
                self.side_3_laser_data.append([laser_data[i], i])  # 左墙
            elif self.angle_3 <= absolute_angle < self.angle_4:
                self.side_4_laser_data.append([laser_data[i], i])  # 下墙
    def calculate_laser_position(self) -> List[float]:
        """使用激光数据计算机器人位置 (核心定位算法)"""
        current_position = [self.robot_x, self.robot_y]
        self.criteria(current_position)
        self.classify_laser_data(self.laser_data)
        # 将self.yaw转换到0-2pi之间
        self.yaw = self.yaw % (np.pi*2)
        # 使用解析解计算角度
        self.a = 0
        self.b = 0
        # 从上墙和下墙的激光数据计算参数
        for dis, id in self.side_2_laser_data:
            self.a += dis * np.cos(self.laser_angel_list[id]) / len(self.side_2_laser_data)
            self.b += dis * np.sin(self.laser_angel_list[id]) / len(self.side_2_laser_data)
        for dis, id in self.side_4_laser_data:
            self.a -= dis * np.cos(self.laser_angel_list[id]) / len(self.side_4_laser_data)
            self.b -= dis * np.sin(self.laser_angel_list[id]) / len(self.side_4_laser_data)
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
        print("a",self.a)
        print("b",self.b)
        # if self.a < 0:
        #     self.c = - self.c
        # angle = np.arcsin(self.c/np.sqrt(self.a**2+self.b**2)) - np.arctan2(self.b,self.a)
        # angle = angle % (np.pi*2)
        # phi = np.arctan2(self.b,self.a)
        # if 0 <= self.yaw < np.pi/2 or np.pi*3/2 <= self.yaw <= np.pi*2:
        #     angle = angle
        # else:
        #     angle = np.pi - angle - 2*phi
        # 用计算出的角度重新计算位置
        list_1 = []  # 右墙距离
        list_2 = []  # 上墙距离  
        list_3 = []  # 左墙距离
        list_4 = []  # 下墙距离
        
        for dis, id in self.side_1_laser_data:
            list_1.append(abs(dis * np.cos(self.laser_angel_list[id] + angle)))
        for dis, id in self.side_2_laser_data:
            list_2.append(abs(dis * np.sin(self.laser_angel_list[id] + angle)))
        for dis, id in self.side_3_laser_data:
            list_3.append(abs(dis * np.cos(self.laser_angel_list[id] + angle)))
        for dis, id in self.side_4_laser_data:
            list_4.append(abs(dis * np.sin(self.laser_angel_list[id] + angle)))
        
        # 计算X坐标
        if len(list_3) == 0:
            x = self.FIELD_SIZE[0] - np.mean(list_1) if len(list_1) > 0 else current_position[0]
        elif len(list_1) == 0:
            x = np.mean(list_3)
        else:
            x = (np.mean(list_3) + self.FIELD_SIZE[0] - np.mean(list_1)) / 2

        # 计算Y坐标
        if len(list_4) == 0:
            y = self.FIELD_SIZE[1] - np.mean(list_2) if len(list_2) > 0 else current_position[1]
        elif len(list_2) == 0:
            y = np.mean(list_4)
        else:
            y = (np.mean(list_4) + self.FIELD_SIZE[1] - np.mean(list_2)) / 2
        print([float(x), float(y), float(angle)],self.yaw)
        return [float(x), float(y), float(angle)]

    def calculate_laser_distances(self):
        """计算每个激光雷达的距离 (仿真用)"""
        distances = []
        
        for i, laser_angle in enumerate(self.laser_angel_list):
            # 计算激光方向 (相对于机器人朝向)
            absolute_angle = self.robot_angle + laser_angle
            dx = math.cos(absolute_angle)
            dy = math.sin(absolute_angle)
            
            # 从机器人位置发射激光，检查与场地边界的交点
            min_distance = self.laser_max_range
            
            # 检查与场地边界的交点
            boundaries = [
                [(0, 0), (self.field_width, 0)],  # 下边界
                [(self.field_width, 0), (self.field_width, self.field_height)],  # 右边界
                [(self.field_width, self.field_height), (0, self.field_height)],  # 上边界
                [(0, self.field_height), (0, 0)],  # 左边界
            ]
            
            for boundary in boundaries:
                start, end = boundary
                intersection_dist = self.line_intersection_distance(
                    self.robot_x, self.robot_y, dx, dy,
                    start[0], start[1], end[0], end[1]
                )
                if intersection_dist is not None and intersection_dist < min_distance:
                    min_distance = intersection_dist
            
            # 添加距离偏置
            distances.append(max(0.1, min_distance + self.DELTA_DISTANCE))
            
        return distances
    
    def line_intersection_distance(self, px, py, dx, dy, x1, y1, x2, y2):
        """计算射线与线段的交点距离"""
        # 射线: P + t*D, 其中 P=(px,py), D=(dx,dy), t>=0
        # 线段: (x1,y1) 到 (x2,y2)
        
        # 线段方向向量
        lx = x2 - x1
        ly = y2 - y1
        
        # 求解方程组
        denom = dx * ly - dy * lx
        if abs(denom) < 1e-10:  # 平行线
            return None
            
        # 参数t (射线参数)
        t = ((x1 - px) * ly - (y1 - py) * lx) / denom
        
        # 参数s (线段参数)
        if abs(lx) > abs(ly):
            s = (px + t * dx - x1) / lx
        else:
            s = (py + t * dy - y1) / ly
            
        # 检查交点是否在射线上 (t>=0) 和线段上 (0<=s<=1)
        if t >= 0 and 0 <= s <= 1:
            return t
        return None
    
    def update_plot(self):
        """更新图形显示"""
        self.ax.clear()
        self.ax.set_xlim(0, self.field_width)
        self.ax.set_ylim(0, self.field_height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Robot Laser Positioning Simulation - Left: move, Right: rotate, A/D(±5°), Q/E(±1°)')
        
        # 绘制场地边界
        boundary_x = [0, self.field_width, self.field_width, 0, 0]
        boundary_y = [0, 0, self.field_height, self.field_height, 0]
        self.ax.plot(boundary_x, boundary_y, 'k-', linewidth=3)
        
        # 更新机器人真实朝向角 (与robot_angle同步)
        self.yaw = self.robot_angle
        
        # 计算激光距离数据
        self.laser_data = self.calculate_laser_distances()
        
        # 使用定位算法计算位置
        try:
            self.calculated_position = self.calculate_laser_position()
            # 计算定位误差
            actual_pos = [self.robot_x, self.robot_y]
            calc_pos = [self.calculated_position[0], self.calculated_position[1]]
            self.position_error = math.sqrt((actual_pos[0] - calc_pos[0])**2 + (actual_pos[1] - calc_pos[1])**2)
        except Exception as e:
            print("----------e------------",e)
            self.calculated_position = [self.robot_x, self.robot_y, self.robot_angle]
            self.position_error = 0.0
        
        # 绘制真实机器人位置 (蓝色)
        robot_vertices = self.get_robot_vertices()
        robot_patch = patches.Polygon(robot_vertices, closed=True, 
                                    facecolor='blue', edgecolor='darkblue', alpha=0.7)
        self.ax.add_patch(robot_patch)
        
        # 绘制计算出的机器人位置 (红色)
        if self.calculated_position:
            calc_vertices = self.get_robot_vertices(
                self.calculated_position[0], 
                self.calculated_position[1], 
                self.calculated_position[2]
            )
            calc_patch = patches.Polygon(calc_vertices, closed=True, 
                                       facecolor='red', edgecolor='darkred', alpha=0.4)
            self.ax.add_patch(calc_patch)
        
        # 绘制机器人朝向箭头 (真实位置 - 蓝色)
        arrow_length = self.robot_size * 0.8
        self.ax.arrow(self.robot_x, self.robot_y, 
                     arrow_length * math.cos(self.robot_angle),
                     arrow_length * math.sin(self.robot_angle),
                     head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        # 绘制计算位置的朝向箭头 (红色)
        if self.calculated_position:
            self.ax.arrow(self.calculated_position[0], self.calculated_position[1], 
                         arrow_length * math.cos(self.calculated_position[2]),
                         arrow_length * math.sin(self.calculated_position[2]),
                         head_width=0.1, head_length=0.1, fc='red', ec='red', alpha=0.7)
        
        # 计算并绘制激光
        laser_info = []
        colors = ['red', 'orange', 'yellow', 'green', 'purple']  # 不同激光用不同颜色
        
        for i, (laser_angle, distance) in enumerate(zip(self.laser_angel_list, self.laser_data)):
            absolute_angle = self.robot_angle + laser_angle
            end_x = self.robot_x + distance * math.cos(absolute_angle)
            end_y = self.robot_y + distance * math.sin(absolute_angle)
            
            # 绘制激光束
            self.ax.plot([self.robot_x, end_x], [self.robot_y, end_y], 
                        color=colors[i], linewidth=2, alpha=0.6)
            
            # 绘制激光终点
            self.ax.plot(end_x, end_y, 'o', color=colors[i], markersize=5)
            
            # Collect laser information
            laser_info.append(f"Laser{i+1}: {distance:.2f}m")
        
        # Display classification information
        classification_info = []
        if hasattr(self, 'side_1_laser_data'):
            # Right wall laser details
            if len(self.side_1_laser_data) > 0:
                side1_details = [f"Laser{data[1]+1}({data[0]:.2f}m)" for data in self.side_1_laser_data]
                classification_info.append(f"Right wall({len(self.side_1_laser_data)}): {', '.join(side1_details)}")
            else:
                classification_info.append("Right wall(0): None")
            
            # Top wall laser details
            if len(self.side_2_laser_data) > 0:
                side2_details = [f"Laser{data[1]+1}({data[0]:.2f}m)" for data in self.side_2_laser_data]
                classification_info.append(f"Top wall({len(self.side_2_laser_data)}): {', '.join(side2_details)}")
            else:
                classification_info.append("Top wall(0): None")
            
            # Left wall laser details
            if len(self.side_3_laser_data) > 0:
                side3_details = [f"Laser{data[1]+1}({data[0]:.2f}m)" for data in self.side_3_laser_data]
                classification_info.append(f"Left wall({len(self.side_3_laser_data)}): {', '.join(side3_details)}")
            else:
                classification_info.append("Left wall(0): None")
            
            # Bottom wall laser details
            if len(self.side_4_laser_data) > 0:
                side4_details = [f"Laser{data[1]+1}({data[0]:.2f}m)" for data in self.side_4_laser_data]
                classification_info.append(f"Bottom wall({len(self.side_4_laser_data)}): {', '.join(side4_details)}")
            else:
                classification_info.append("Bottom wall(0): None")
        
        # 显示信息
        angle_deg = math.degrees(self.robot_angle) % 360
        calc_angle_deg = math.degrees(self.calculated_position[2]) % 360 if self.calculated_position else 0.0
        
        info_text = f"Real Position: ({self.robot_x:.2f}, {self.robot_y:.2f}) Angle: {angle_deg:.1f}°\n"
        if self.calculated_position:
            info_text += f"Calculated Position: ({self.calculated_position[0]:.2f}, {self.calculated_position[1]:.2f}) Angle: {calc_angle_deg:.1f}°\n"
            info_text += f"Position Error: {self.position_error:.3f}m\n"
        info_text += "Control: A/D(±5°), Q/E(±1°)\n"
        info_text += "Blue=Real Position, Red=Calculated Position\n\n"
        
        # Display angle thresholds
        if hasattr(self, 'angle_1') and self.angle_1 is not None:
            info_text += f"Angle Thresholds: θ1={math.degrees(self.angle_1):.1f}° θ2={math.degrees(self.angle_2):.1f}° θ3={math.degrees(self.angle_3):.1f}° θ4={math.degrees(self.angle_4):.1f}°\n\n"
        
        info_text += "\n".join(laser_info)
        if classification_info:
            info_text += "\n\n" + "\n".join(classification_info)
        
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", 
                    facecolor="white", alpha=0.8))
        
        self.fig.canvas.draw()
    
    def get_robot_vertices(self, x=None, y=None, angle=None):
        """获取机器人多边形顶点 (三角形表示朝向)"""
        if x is None:
            x = self.robot_x
        if y is None:
            y = self.robot_y
        if angle is None:
            angle = self.robot_angle
            
        # 三角形顶点 (朝向为前方)
        local_vertices = np.array([
            [self.robot_size, 0],      # 前端
            [-self.robot_size/2, self.robot_size/2],   # 左后
            [-self.robot_size/2, -self.robot_size/2]   # 右后
        ])
        
        # 旋转
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        
        rotated_vertices = np.dot(local_vertices, rotation_matrix.T)
        
        # 平移到机器人位置
        rotated_vertices[:, 0] += x
        rotated_vertices[:, 1] += y
        
        return rotated_vertices
    
    def on_mouse_press(self, event):
        """鼠标按下事件"""
        if event.inaxes != self.ax:
            return
            
        # 检查是否点击在机器人附近
        dist = math.sqrt((event.xdata - self.robot_x)**2 + (event.ydata - self.robot_y)**2)
        if dist <= self.robot_size * 2:
            if event.button == 1:  # 左键拖拽移动
                self.dragging = True
            elif event.button == 3:  # 右键拖拽旋转
                self.rotating = True
            self.last_mouse_pos = (event.xdata, event.ydata)
    
    def on_mouse_release(self, event):
        """鼠标释放事件"""
        self.dragging = False
        self.rotating = False
        self.last_mouse_pos = None
    
    def on_mouse_motion(self, event):
        """鼠标移动事件"""
        if event.inaxes != self.ax or self.last_mouse_pos is None:
            return
            
        if self.dragging and event.xdata is not None and event.ydata is not None:
            # 移动机器人
            # 限制在场地范围内
            new_x = max(0.5, min(self.field_width - 0.5, event.xdata))
            new_y = max(0.5, min(self.field_height - 0.5, event.ydata))
            
            self.robot_x = new_x
            self.robot_y = new_y
            self.update_plot()
            
        elif self.rotating and event.xdata is not None and event.ydata is not None:
            # 旋转机器人
            dx = event.xdata - self.robot_x
            dy = event.ydata - self.robot_y
            self.robot_angle = math.atan2(dy, dx)
            self.update_plot()
        
        if event.xdata is not None and event.ydata is not None:
            self.last_mouse_pos = (event.xdata, event.ydata)
    
    def on_key_press(self, event):
        """键盘按键事件"""
        if event.key == 'a' or event.key == 'A':
            # A键逆时针旋转5度
            self.robot_angle -= math.radians(5)
            self.update_plot()
        elif event.key == 'd' or event.key == 'D':
            # D键顺时针旋转5度
            self.robot_angle += math.radians(5)
            self.update_plot()
        elif event.key == 'q' or event.key == 'Q':
            # Q键逆时针旋转1度 (精细调节)
            self.robot_angle -= math.radians(1)
            self.update_plot()
        elif event.key == 'e' or event.key == 'E':
            # E键顺时针旋转1度 (精细调节)
            self.robot_angle += math.radians(1)
            self.update_plot()

def main():
    """Main function"""
    print("Starting Robot Laser Positioning Simulation...")
    print("Controls:")
    print("- Left click & drag: Move robot")
    print("- Right click & drag: Rotate robot")
    print("- A/D keys: 5° angle adjustment (counter-clockwise/clockwise)")
    print("- Q/E keys: 1° fine angle adjustment (counter-clockwise/clockwise)")
    print("- Blue=Real Position, Red=Calculated Position")
    print("- Close window to exit")
    
    simulator = RobotLaserSimulator()
    plt.show()

if __name__ == "__main__":
    main()

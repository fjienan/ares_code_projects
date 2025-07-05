# ares_code_projects

**首先我们必须统一我们所认为的世界坐标系是什么**

                                ----------------
                                |              |
                                |              |
                                |              |
                                |              |
                                |              |
                            15  |              |
                                |              |
                                |              |
                                |              |
                                |              |     ^
                                |              |     |x
                                ----------------  <---  
                                        8           y

## 最新更新
- 2025.6.25 更新了五连杆通信的代码，增加了usb_bulk_node包
## tree
        .
        ├──transformation.py
        │ 
        ├── R1
        │   ├── build
        │   ├── install
        │   ├── log
        │   └── src
        │       ├── laser
        │       ├── my_robot_bringup
        │       ├── myrobot_controller
        │       ├── my_robot_description
        │       ├── omni_mpc_controller
        │       ├──remote_rc_node
        │       ├── sim_localization
        │       ├── sllidar_ros2
        │       └── usb_bulk_node #五连杆的代码暂时放在这里
        ├── R2
        │   └── usb_bulk_node #五连杆的代码暂时放在这里
        │
        └── README.md
## 说明
### omni_mpc_controller
执行launch 
``` terminal
ros2 launch omni_mpc_controller MPC.launch.py ##启动避障程序
ros2 launch omni_mpc_controller keyboard.launch.py ##启动键盘控制
```
topic消息输出
```
ros2 topic list ##列出所有topic
ros2 topic echo /xxx ##输出topic消息
ros2 topic info /xxx ##输出topic的发布者数目以及当前订阅者数目
```
### 摇感启动
#### Steamdeck
和电脑连同一个wifi
**terminal_1**
``` terminal
sudo openhd -g
```
#### 电脑
**terminal_1**
``` ternimal
sudo openhd -a
```
```terminal
ros2 run remote_rc_node udp_joy_node
```
```terminal
ros2 run usb_bulk_node usb_bulk_node
```
## ROS2 Humble tiny tricks
### 1.重映射：实现在不修改代码的情况下将原来的topic改在其他名称的topic上发布
#### Terminal :
``` terminal
ros2 run (package_name) (node_name) --ros-args -r (origin_topic_name):=(new_topic_name)
```

#### Launch :
``` python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            remappings=[
                ('/chatter', '/conversation')
            ],
            output='screen'
        )
    ])
```

ros2 run omni_mpc_controller keyboard_control --ros-args --param-file /home/furry/ares_code_projects/src/config/params.yaml

## laser
1 初始化3d雷达
```
zsh
map
```
```
ros2 launch laser all_nodes.launch.py
```
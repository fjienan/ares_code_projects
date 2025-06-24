# ares_code_projects

## tree
        .
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
        │       ├── sim_localization
        │       ├── sllidar_ros2
        │       └── usb_bulk_node
        ├── R2
        │   ├── build
        │   ├── install
        │   ├── log
        │   │   
        │   └── src
        │       ├── laser
        │       ├── my_robot_bringup
        │       ├── myrobot_controller
        │       ├── my_robot_description
        │       ├── omni_mpc_controller
        │       ├── sim_localization
        │       ├── sllidar_ros2
        │       └── usb_bulk_node
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

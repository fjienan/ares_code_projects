from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    ld = LaunchDescription()
     
    pkg_dir = get_package_share_directory('omni_mpc_controller')
    # 指定参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    mpc_node = Node(
        package = 'omni_mpc_controller',
        executable = 'MPC',
        output = 'screen',
        parameters = [params_file]
    )
    ld.add_action(mpc_node)

    #单点激光定位模块
    
    # pkg_dir_2 = get_package_share_directory('laser')
    # params_file_2 = os.path.join(pkg_dir_2, 'config', 'params.yaml')
    # myRreal_node = Node(
    #     package='laser',
    #     executable='Real',
    #     # name='myRreal_node',
    #     output='screen',
    #     parameters=[params_file_2] # 加载参数文件
    # )    
    # ld.add_action(myRreal_node)

    return ld
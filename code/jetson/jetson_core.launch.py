import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 1. Start Lidara
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
        )
    )

    
    # 3. Mostek UART (komunikacja z STM32)
    uart_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(current_dir, 'uart_bridge.py')],
        output='screen'
    )

    return LaunchDescription([
        lidar_launch,
        uart_bridge
    ])

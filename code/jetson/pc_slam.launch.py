import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # 1. Odometria z Lidara (rf2o)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }]
    )

    # 2. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'odom_frame': 'odom'},
            {'map_frame': 'map'},
            {'base_frame': 'base_link'},
            {'scan_topic': '/scan'},
            {'mode': 'mapping'},
            {'minimum_travel_distance': 0.15},
            {'minimum_travel_heading': 0.2},
            {'map_update_interval': 2.0},
            {'resolution': 0.01}
        ]
    )

    # 3. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(current_dir, 'slam_config.rviz')],
        output='screen'
    )

    # 4. Automatyczne nagrywanie trasy (rosbag)
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', 'moje_nagranie_slam', '/scan', '/tf', '/tf_static', '/odom', '/map'],
        output='screen'
    )

    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    return LaunchDescription([
        rf2o_node,
        slam_toolbox,
        rviz,
        record_bag,
	tf_base_to_laser
    ])

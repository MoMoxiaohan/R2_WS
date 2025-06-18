from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('point_lio')), '/launch','/point_lio.launch.py']),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'aft_mapped', '--child-frame-id', 'laser'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_registered'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'laser',
                'transform_tolerance': 0.1,
                'min_height': 0.1, # -20cm
                'max_height': 0.5,  #  20cm
                'angle_min': -3.1415,  # - M_PI
                'angle_max': 3.1415,  # M_PI
                'angle_increment': 0.0174,  # M_PI * 2 / 360.0 = 1 degree
                'scan_time': 0.1,# 10Hz
                'range_min': 0.1, # 10cm
                'range_max': 20.0, # 70m
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_transform_publisher',
        #     # parameters=[{'x':0}, {'y': 0}, {'z': 0}, {'yaw': 0}, {'pitch': 0}, {'roll': 0},
        #     #           {'frame_id': "body"}, {'child_frame_id': "laser"}]
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'camera_init'
        #     ]
        # ),
    ])
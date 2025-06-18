# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import UnlessCondition
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     # 声明 launch 参数
#     use_sim_time = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation (Gazebo) clock if true'
#     )
    
#     map_file = DeclareLaunchArgument(
#         '/home/liu/vinci_projects/lio_localization_ws/src/slam_nav/maps/map2',
#         default_value=PathJoinSubstitution([
#             FindPackageShare('slam_nav'), 
#             'maps', 
#             'slam_map'
#         ]),
#         description='Path to save the map file'
#     )
    
#     # SLAM Toolbox 节点
#     slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='sync_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[{
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
            
#             # 帧参数
#             'base_frame': 'aft_mapped',
#             'odom_frame': 'camera_init',
#             'map_frame': 'map',
#             'scan_topic': '/scan',
#             'map_start_pose': [0.0, 0.0, 0.0],
#             'map_start_at_dock': True,
            
#             # 模式配置
#             'mode': 'mapping',  # 建图模式
#             'enable_interactive_mode': True,  # 允许交互式模式
#             'use_scan_matching': True,
#             # 地图保存配置
#             'map_file_name': LaunchConfiguration("/home/liu/vinci_projects/lio_localization_ws/src/slam_nav/maps/map2"),
#             'save_map_timeout': 500.0,  # 保存地图超时(ms)
            
#             # 扫描匹配参数
#             'resolution': 0.05,  # 地图分辨率 (m/cell)
#             'max_laser_range': 20.0,  # 最大激光范围(m)
#             'minimum_time_interval': 0.5,  # 最小处理间隔(s)
#             'transform_publish_period': 0.02,  # 坐标发布周期(s)
            
#             # 优化参数
#             'optimizer_params': {
#                 'type': 'sparse_normal_cholesky',
#                 'lambda': 1.0,
#                 'max_iterations': 12,
#                 'early_stopping': True
#             },
            
#             # 闭环检测参数
#             'loop_search_maximum_distance': 5.0,
#             'loop_match_minimum_chain_size': 10,
#             'loop_match_maximum_variance_coarse': 3.0
#         }]
#     )
    
#     return LaunchDescription([

#         use_sim_time,
#         #open_rviz,
#         map_file,
#         slam_toolbox_node,
#         #rviz_node
#     ])
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_nav"),
                                   'config', 'mapper_params_online_sync.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_sync_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    map_tf_node=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'camera_init'
            ]
        )
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(map_tf_node)

    return ld
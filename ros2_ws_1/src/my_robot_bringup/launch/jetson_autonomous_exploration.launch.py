"""
Jetson Xavier için Otonom Keşif Launch Dosyası
Dağıtık yapı için: Controller'lar + SLAM + Nav2 + Explore (Gazebo ve RViz2 olmadan)

Bu dosya sadece Jetson'da çalıştırılmalı.
Bilgisayarda Gazebo çalışıyor olmalı.

Kullanım:
ros2 launch my_robot_bringup jetson_autonomous_exploration.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    
    # Konfigürasyon dosyası yolları
    nav2_config_path = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    slam_config_path = os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml')
    explore_config_path = os.path.join(pkg_bringup, 'config', 'explore_params.yaml')
    twist_mux_config_path = os.path.join(pkg_bringup, 'config', 'twist_mux.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # 1. Controller Spawner'lar (0 saniye - anında başlar)
    diff_drive_spawner = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    joint_broad_spawner = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # 2. Twist Mux (2 saniye sonra)
    twist_mux = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_config_path, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
                output='screen'
            )
        ]
    )
    
    # 3. SLAM Toolbox (4 saniye sonra)
    slam_toolbox_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'slam_params_file': slam_config_path,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )
    
    # 4. Nav2 Navigation Stack (6 saniye sonra)
    nav2_navigation = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': nav2_config_path
                }.items()
            )
        ]
    )
    
    # 5. m-explore-ros2 - Otonom Keşif (12 saniye sonra - SLAM ve Nav2'nin hazır olması için)
    explore_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore',
                output='screen',
                parameters=[explore_config_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/map', '/map'),  # SLAM'dan gelen harita
                    ('/scan', '/lidar/out'),  # LIDAR verisi
                ]
            )
        ]
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        declare_autostart_arg,
        
        diff_drive_spawner,  # 1 saniye sonra
        joint_broad_spawner, # 1 saniye sonra
        twist_mux,           # 2 saniye sonra
        slam_toolbox_launch, # 4 saniye sonra
        nav2_navigation,      # 6 saniye sonra
        explore_node,         # 12 saniye sonra
    ])


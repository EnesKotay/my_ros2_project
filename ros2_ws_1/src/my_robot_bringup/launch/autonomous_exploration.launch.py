"""
Otonom SLAM Keşif Launch Dosyası
Teknofest Sanayide Dijital Teknolojiler Yarışması için

Bu launch dosyası:
1. SLAM Toolbox ile harita oluşturur
2. Nav2 ile navigasyon sağlar
3. m-explore-ros2 ile otonom keşif yapar

Kullanım:
ros2 launch my_robot_bringup autonomous_exploration.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
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
    
    # Gazebo launch dosyası
    gazebo_launch_path = os.path.join(pkg_bringup, 'launch', 'my_robot_gazebo.launch.xml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # 1. Gazebo ve Robot State Publisher'ı Başlat
    gazebo_and_robot = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. Spawn Entity - Robot'u Gazebo'ya spawn et (3 saniye sonra)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                    '-timeout', '30'
                ],
                output='screen'
            )
        ]
    )
    
    # 3. Controller Spawner'ları (5 saniye sonra)
    diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad", "--controller-manager", "/controller_manager"],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 4. Twist Mux (6 saniye sonra)
    twist_mux = TimerAction(
        period=6.0,
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
    
    # 5. SLAM Toolbox - Harita oluşturma (8 saniye sonra)
    slam_toolbox_launch = TimerAction(
        period=8.0,
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
    
    # 6. Nav2 Navigation Stack (9 saniye sonra)
    nav2_navigation = TimerAction(
        period=9.0,
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
    
    # 7. m-explore-ros2 - Otonom Keşif (18 saniye sonra - SLAM'ın harita oluşturması için yeterli zaman)
    # NOT: m-explore-ros2 paketini workspace'inize eklemeniz gerekiyor
    # cd ~/Desktop/ros2_ws_1/src
    # git clone https://github.com/robo-friends/m-explore-ros2.git
    # cd ~/Desktop/ros2_ws_1
    # colcon build --packages-select explore_lite
    explore_node = TimerAction(
        period=18.0,  # Artırıldı: SLAM'ın harita oluşturması ve Nav2'nin hazır olması için (12.0 -> 18.0)
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
    
    # 8. RViz2 - Görselleştirme (20 saniye sonra)
    rviz_node = TimerAction(
        period=20.0,  # Artırıldı: Explore node'un başlamasından sonra (15.0 -> 20.0)
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        
        gazebo_and_robot,
        spawn_entity,
        twist_mux,
        diff_drive_spawner,
        joint_broad_spawner,
        slam_toolbox_launch,
        nav2_navigation,
        explore_node,  # Otonom keşif node'u
        rviz_node,
    ])


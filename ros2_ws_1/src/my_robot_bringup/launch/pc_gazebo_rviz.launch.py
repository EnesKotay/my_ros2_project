"""
Bilgisayar (PC) için Gazebo + RViz2 Launch Dosyası
Dağıtık yapı için: Sadece simülasyon ve görselleştirme

Bu dosya sadece bilgisayarında çalıştırılmalı.
Jetson'da çalıştırma - Gazebo ve RViz2 yüklü değil.

Kullanım:
ros2 launch my_robot_bringup pc_gazebo_rviz.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare('my_robot_bringup')
    pkg_description = FindPackageShare('my_robot_description')
    
    # Konfigürasyon dosyası yolları
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'main.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_bringup, 'rviz', 'urdf_config.rviz'])
    gazebo_launch_path = PathJoinSubstitution([pkg_bringup, 'launch', 'my_robot_gazebo.launch.xml'])
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 1. Gazebo ve Robot State Publisher'ı başlat (mevcut XML launch dosyasını kullan)
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
    
    # 3. RViz2 - Görselleştirme (5 saniye sonra)
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        
        gazebo_and_robot,  # 0 saniye - anında başlar (Gazebo + Robot State Publisher)
        spawn_entity,      # 3 saniye sonra
        rviz_node,         # 5 saniye sonra
    ])


"""
Jetson Xavier için Kontrol Launch Dosyası
Dağıtık yapı için: Sadece controller'lar ve twist_mux

Bu dosya sadece Jetson'da çalıştırılmalı.
Bilgisayarda Gazebo çalışıyor olmalı.

Kullanım:
ros2 launch my_robot_bringup jetson_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare('my_robot_bringup')
    
    # Konfigürasyon dosyası yolları
    twist_mux_config_path = PathJoinSubstitution([pkg_bringup, 'config', 'twist_mux.yaml'])
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 1. Controller Spawner'lar - Robot kontrolcülerini aktif eder
    # NOT: Controller manager bilgisayardaki Gazebo tarafından sağlanıyor
    # Bu node'lar sadece controller'ları spawn ediyor
    
    diff_drive_spawner = TimerAction(
        period=2.0,  # Gazebo'nun hazır olması için kısa bir bekleme
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
        period=2.0,
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
    
    # 2. Twist Mux - Nav2'den gelen cmd_vel'i robot controller'ına yönlendirir
    twist_mux = TimerAction(
        period=3.0,  # Controller'ların hazır olması için bekleme
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
    
    return LaunchDescription([
        declare_sim_time_arg,
        
        diff_drive_spawner,  # 2 saniye sonra
        joint_broad_spawner, # 2 saniye sonra
        twist_mux,           # 3 saniye sonra
    ])


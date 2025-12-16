from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup') 
    
    # 1. Launch Argümanlarını Tanımlama (Esneklik için)
    # Bu, simülasyon zamanını tüm düğümlere geçirecektir.
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True', # Simülasyonda olduğunuz için True varsayılıyor
        description='Kullanılacak simülasyon zamanı (True/False)'
    )

    # SLAM Toolbox parametre dosyasının yolu
    slam_config_path = PathJoinSubstitution([pkg_bringup, 'config', 'mapper_params_online_async.yaml'])
    
    # Simülasyon Launch dosyasının yolu (varsayımsal Python dosyası)
    robot_sim_launch_path = PathJoinSubstitution([pkg_bringup, 'launch', 'robot_sim.launch.py'])
    
    # 2. Gazebo ve Robot Modeli Başlatma (Python dosyası dahil ediliyor)
    gazebo_and_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_sim_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. SLAM Toolbox Düğümünü Tanımla
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        output='log', # Ekran çıktısını temiz tutmak için 'log' kullanıldı
        parameters=[
            slam_config_path, 
            {'use_sim_time': use_sim_time}
        ],
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        gazebo_and_robot_launch,
        slam_node,
    ])
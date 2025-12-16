from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Paket Dizinleri ve Konfigürasyon Dosyası Yolları ---
    
    # Kendi paketimizden dosya almak için PathJoinSubstitution kullanıyoruz
    pkg_bringup = FindPackageShare('my_robot_bringup')
    
    # Navigasyon, SLAM, Twist Mux ve Rviz konfigürasyon yolları (Taşınabilir)
    nav2_config_path = PathJoinSubstitution([pkg_bringup, 'config', 'nav2_params.yaml'])
    slam_config_path = PathJoinSubstitution([pkg_bringup, 'config', 'mapper_params_online_async.yaml'])
    twist_mux_config_path = PathJoinSubstitution([pkg_bringup, 'config', 'twist_mux.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_bringup, 'rviz', 'urdf_config.rviz'])
    
    # Gazebo launch dosyası (XML yerine Python dosyasına geçiş şiddetle önerilir)
    # Şimdilik XML'i koruyoruz, ancak yorumda Python'a geçiş yolu belirtilmiştir.
    gazebo_launch_path = PathJoinSubstitution([pkg_bringup, 'launch', 'my_robot_gazebo.launch.xml'])
    
    # --- 2. Launch Arguments ---
    
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
    
    # --- 3. Başlatma Aksiyonları ---
    
    # 3.1. Gazebo ve Robot'u Başlat
    # NOT: FrontendLaunchDescriptionSource (XML) yerine PythonLaunchDescriptionSource kullanılması profesyonel standarttır.
    gazebo_and_robot = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(gazebo_launch_path),
        # Gazebo launch dosyası içinde use_sim_time'ı ayarlamak önemlidir
        launch_arguments={'use_sim_time': use_sim_time}.items() 
    )
    
    # 3.2. Twist Mux (Simülasyon zamanı argüman ile besleniyor)
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_path, {'use_sim_time': use_sim_time}], # Düzeltme
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        output='log' # Konsol çıktısını temiz tutar
    )
    
    # 3.3. Controller Spawner'lar (Simülasyon zamanı argüman ile besleniyor)
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3.4. SLAM Toolbox (Modüler Include)
    slam_toolbox_launch = IncludeLaunchDescription(
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
    
    # 3.5. Nav2 Navigation Stack (Modüler Include)
    nav2_navigation = IncludeLaunchDescription(
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
    
    # 3.6. RViz2 (Taşınabilir Rviz konfigürasyonu kullanılıyor)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], # Kendi rviz dosyanız yüklendi
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # --- 4. Launch Tanımını Döndür ---
    return LaunchDescription([
        declare_sim_time_arg,
        declare_autostart_arg,
        
        gazebo_and_robot,
        
        # Controller Spawner'lar (Gazebo'nun başlama sırasına dikkat edilmeli)
        diff_drive_spawner,
        joint_broad_spawner,
        
        twist_mux,
        slam_toolbox_launch,
        nav2_navigation,
        rviz_node,
    ])
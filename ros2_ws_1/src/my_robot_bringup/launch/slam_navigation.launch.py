from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
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
    
    # 3.1. Gazebo ve Robot'u Başlat (0 saniye - anında)
    # NOT: FrontendLaunchDescriptionSource (XML) yerine PythonLaunchDescriptionSource kullanılması profesyonel standarttır.
    gazebo_and_robot = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(gazebo_launch_path),
        # Gazebo launch dosyası içinde use_sim_time'ı ayarlamak önemlidir
        launch_arguments={'use_sim_time': use_sim_time}.items() 
    )
    
    # 3.2. Spawn Entity - Robot'u Gazebo'ya spawn et (3 saniye sonra)
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
    
    # 3.3. Controller Spawner'lar (5 saniye sonra)
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
    
    # 3.4. Twist Mux (6 saniye sonra)
    twist_mux = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_config_path, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
                output='log'
            )
        ]
    )
    
    # 3.5. SLAM Toolbox (8 saniye sonra)
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
    
    # 3.6. Nav2 Navigation Stack (9 saniye sonra)
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
    
    # 3.7. RViz2 (13 saniye sonra)
    rviz_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # --- 4. Launch Tanımını Döndür ---
    return LaunchDescription([
        declare_sim_time_arg,
        declare_autostart_arg,
        
        gazebo_and_robot,  # 0 saniye - anında başlar
        spawn_entity,  # 3 saniye sonra
        diff_drive_spawner,  # 5 saniye sonra
        joint_broad_spawner,  # 5 saniye sonra
        twist_mux,  # 6 saniye sonra
        slam_toolbox_launch,  # 8 saniye sonra
        nav2_navigation,  # 9 saniye sonra
        rviz_node,  # 13 saniye sonra
    ])
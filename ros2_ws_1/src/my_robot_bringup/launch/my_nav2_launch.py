from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
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
    nav2_config_path = os.path.join(
        pkg_bringup,
        'config',
        'nav2_params.yaml'
    )
    
    # Harita dosyası yolu (workspace root'ta)
    workspace_root = os.path.join(
        os.path.expanduser('~'),
        'Desktop',
        'ros2_ws_1'
    )
    map_file_path = os.path.join(workspace_root, 'ilk_map.yaml')
    
    twist_mux_config_path = os.path.join(
        pkg_bringup,
        'config',
        'twist_mux.yaml'
    )
    
    # Gazebo launch dosyası
    gazebo_launch_path = os.path.join(
        pkg_bringup,
        'launch',
        'my_robot_gazebo.launch.xml'
    )
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # 1. Gazebo ve Robot State Publisher'ı Başlat
    gazebo_and_robot = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. Spawn Entity - Robot'u Gazebo'ya spawn et
    # Gazebo başladıktan sonra spawn etmek için TimerAction kullanıyoruz
    # Timeout artırıldı ve robot zaten varsa hata vermemesi için -timeout parametresi eklendi
    spawn_entity = TimerAction(
        period=3.0,  # Gazebo başladıktan 3 saniye sonra spawn et
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
                    '-timeout', '30'  # Timeout 30 saniyeye çıkarıldı
                ],
                output='screen'
            )
        ]
    )
    
    # 3. Controller Spawner'ları - Robot kontrolcülerini aktif eder (5 saniye sonra)
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
    
    # 4. Twist Mux - Nav2'den gelen cmd_vel'i robot controller'ına yönlendirir (6 saniye sonra)
    twist_mux = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_config_path, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
            )
        ]
    )
    
    # 5. Map Server - Haritayı yükler (8 saniye sonra)
    map_server = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file_path
                }],
                remappings=[('/tf', 'tf'),
                            ('/tf_static', 'tf_static')]
            )
        ]
    )
    
    # 6. AMCL - Lokalizasyon (9 saniye sonra)
    amcl = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_config_path],
                remappings=[('/tf', 'tf'),
                            ('/tf_static', 'tf_static'),
                            ('/scan', '/lidar/out')]
            )
        ]
    )
    
    # Lifecycle Manager - AMCL ve Map Server'ı yönetir (10 saniye sonra)
    lifecycle_nodes = ['map_server', 'amcl']
    lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': lifecycle_nodes
                }]
            )
        ]
    )
    
    # 7. Nav2 Navigation Stack (12 saniye sonra)
    nav2_navigation = TimerAction(
        period=12.0,
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
    
    # 8. RViz2 (15 saniye sonra)
    rviz_node = TimerAction(
        period=15.0,
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
        
        gazebo_and_robot,  # 0 saniye - anında başlar
        spawn_entity,  # 3 saniye sonra
        diff_drive_spawner,  # 5 saniye sonra
        joint_broad_spawner,  # 5 saniye sonra
        twist_mux,  # 6 saniye sonra
        map_server,  # 8 saniye sonra
        amcl,  # 9 saniye sonra
        lifecycle_manager,  # 10 saniye sonra
        nav2_navigation,  # 12 saniye sonra
        rviz_node,  # 15 saniye sonra
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    # GitHub repo örneğine göre: https://github.com/taherfattahi/ros2-slam-auto-navigation
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )
    
    # 3. Twist Mux - Nav2'den gelen cmd_vel'i robot controller'ına yönlendirir
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_path, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )
    
    # 4. Controller Spawner'ları - Robot kontrolcülerini aktif eder
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
    
    # 5. Map Server - Haritayı yükler
    map_server = Node(
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
    
    # 6. AMCL - Lokalizasyon
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config_path],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/scan', '/lidar/out')]
    )
    
    # Lifecycle Manager - AMCL ve Map Server'ı yönetir
    lifecycle_nodes = ['map_server', 'amcl']
    lifecycle_manager = Node(
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
    
    # 7. Nav2 Navigation Stack
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
    
    # 8. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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
        spawn_entity,  # Spawn entity, Gazebo başladıktan sonra çalışır
        twist_mux,
        diff_drive_spawner,
        joint_broad_spawner,
        map_server,
        amcl,
        lifecycle_manager,
        nav2_navigation,
        rviz_node,
    ])

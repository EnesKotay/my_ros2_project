# ROS2 Navigation Robot with AMCL

ROS2 Humble ile otonom navigasyon robotu projesi. Gazebo simülasyonu, AMCL lokalizasyonu ve Nav2 navigasyon stack'i kullanılmaktadır.

## Özellikler

- **Gazebo Simülasyonu**: Robot ve dünya simülasyonu
- **ROS2 Control**: Differential drive controller ile robot kontrolü
- **AMCL Lokalizasyonu**: Adaptive Monte Carlo Localization
- **Nav2 Navigation Stack**: Otonom navigasyon ve path planning
- **RViz2 Görselleştirme**: Robot durumu ve navigasyon görselleştirmesi

## Gereksinimler

- ROS2 Humble
- Gazebo
- Nav2
- AMCL
- ROS2 Control

### Kurulum

```bash
# Gerekli paketleri kurun
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-amcl
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-twist-mux
```

## Kullanım

### Workspace'i Build Edin

```bash
cd ~/Desktop/ros2_ws_1
colcon build
source install/setup.bash
```

### Launch Dosyasını Çalıştırın

```bash
ros2 launch my_robot_bringup my_nav2_launch.py
```

Bu komut şunları başlatır:
- Gazebo simülasyonu
- Robot spawn
- ROS2 Control (differential drive controller)
- Map Server
- AMCL lokalizasyonu
- Nav2 Navigation Stack
- RViz2

## Proje Yapısı

```
ros2_ws_1/
├── src/
│   ├── my_robot_bringup/      # Launch dosyaları ve konfigürasyonlar
│   │   ├── launch/
│   │   │   ├── my_nav2_launch.py
│   │   │   └── my_robot_gazebo.launch.xml
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── my_controller.yaml
│   │   │   └── twist_mux.yaml
│   │   └── worlds/
│   │       └── test_world.world
│   └── my_robot_description/   # Robot URDF/XACRO dosyaları
│       └── urdf/
│           ├── main.xacro
│           ├── ros2_control.xacro
│           └── ...
└── ilk_map.yaml                # Harita dosyası
```

## Konfigürasyon

### Controller Ayarları

`src/my_robot_bringup/config/my_controller.yaml` dosyasında:
- `odom_frame_id: odom` - Odometry frame ID
- `base_frame_id: base_footprint` - Base frame ID
- `wheel_separation: 0.45` - Tekerlekler arası mesafe
- `wheel_radius: 0.1` - Tekerlek yarıçapı

### Nav2 Parametreleri

`src/my_robot_bringup/config/nav2_params.yaml` dosyasında Nav2 parametreleri ayarlanabilir.

## Transform Frame Yapısı

```
map
  └── odom (AMCL tarafından yayınlanır)
      └── base_footprint (ROS2 Control tarafından yayınlanır)
          └── base_link
              ├── lidar_link
              ├── camera_link
              └── ...
```

## Sorun Giderme

### Odometry Yayınlanmıyor

- `my_controller.yaml` dosyasında `odom_frame_id: odom` parametresinin olduğundan emin olun
- Controller'ın başarıyla spawn edildiğini kontrol edin: `ros2 service list | grep controller_manager`

### TF Hataları

- `use_sim_time: true` parametresinin tüm node'larda ayarlandığından emin olun
- `transform_tolerance` değerlerini artırabilirsiniz (nav2_params.yaml)

## Lisans

MIT License

## Referanslar

- [ROS2 Navigation2](https://navigation.ros.org/)
- [ROS2 Control](https://control.ros.org/)
- [AMCL](https://wiki.ros.org/amcl)


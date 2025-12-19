# 🤖 ROS2 Otonom SLAM Keşif Robotu

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-orange)](https://github.com/SteveMacenski/slam_toolbox)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation2-purple)](https://navigation.ros.org/)

ROS2 Humble ile otonom SLAM (Simultaneous Localization and Mapping) ve frontier-based keşif yapan robot projesi. Robot, bilinmeyen ortamlarda kendi kendine harita oluşturur ve tüm alanı keşfeder.

## ✨ Özellikler

- 🗺️ **Otonom SLAM**: `slam_toolbox` ile gerçek zamanlı harita oluşturma
- 🔍 **Frontier-Based Exploration**: `m-explore-ros2` ile otonom keşif
- 🤖 **Gazebo Simülasyonu**: Gerçekçi robot ve dünya simülasyonu
- 🎮 **ROS2 Control**: Differential drive controller ile robot kontrolü
- 📍 **Nav2 Navigation Stack**: Otonom navigasyon ve path planning
- 👁️ **RViz2 Görselleştirme**: Robot durumu, harita ve navigasyon görselleştirmesi
- 🛡️ **Güvenli Navigasyon**: Gelişmiş obstacle avoidance ve costmap yönetimi

## 📋 İçindekiler

- [Özellikler](#-özellikler)
- [Gereksinimler](#-gereksinimler)
- [Kurulum](#-kurulum)
- [Kullanım](#-kullanım)
- [Proje Yapısı](#-proje-yapısı)
- [Konfigürasyon](#-konfigürasyon)
- [Launch Dosyaları](#-launch-dosyaları)
- [Sorun Giderme](#-sorun-giderme)

## 🔧 Gereksinimler

- **ROS2 Humble** (Ubuntu 22.04)
- **Gazebo** (simülasyon için)
- **Nav2** (navigasyon için)
- **slam_toolbox** (SLAM için)
- **m-explore-ros2** (otonom keşif için - projeye dahil)

### Paket Kurulumu

```bash
# ROS2 Humble kurulumu (eğer yoksa)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop -y

# Gerekli ROS2 paketleri
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-amcl \
                 ros-humble-slam-toolbox \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-twist-mux \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-robot-localization \
                 ros-humble-robot-state-publisher \
                 -y
```

## 🚀 Kurulum

### 1. Repository'yi Klonlayın

```bash
git clone https://github.com/EnesKotay/my_ros2_project.git
cd my_ros2_project/ros2_ws_1
```

### 2. Workspace'i Build Edin

```bash
# Bağımlılıkları kontrol edin
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Workspace'i build edin
colcon build --symlink-install

# Environment'ı source edin
source install/setup.bash
```

## 🎮 Kullanım

### Otonom SLAM Keşif (Önerilen)

Robotun bilinmeyen bir ortamda kendi kendine harita oluşturması ve keşif yapması için:

```bash
ros2 launch my_robot_bringup autonomous_exploration.launch.py
```

Bu komut şunları başlatır:
- 🎮 Gazebo simülasyonu (test_world.world)
- 🤖 Robot spawn ve ROS2 Control
- 🗺️ SLAM Toolbox (harita oluşturma)
- 🧭 Nav2 Navigation Stack
- 🔍 Explore Lite (otonom keşif)
- 👁️ RViz2 (görselleştirme)

### SLAM ve Navigasyon (Manuel Kontrol)

Harita oluştururken manuel kontrol için:

```bash
ros2 launch my_robot_bringup slam_navigation.launch.py
```

### Sadece SLAM

Sadece harita oluşturmak için:

```bash
ros2 launch my_robot_bringup my_slam_launch.py
```

### Önceden Oluşturulmuş Harita ile Navigasyon

Kaydedilmiş harita ile navigasyon için:

```bash
ros2 launch my_robot_bringup my_nav2_launch.py
```

## 📁 Proje Yapısı

```
ros2_ws_1/
├── src/
│   ├── my_robot_bringup/          # Launch dosyaları ve konfigürasyonlar
│   │   ├── launch/
│   │   │   ├── autonomous_exploration.launch.py  # ⭐ Otonom keşif
│   │   │   ├── slam_navigation.launch.py        # SLAM + Navigasyon
│   │   │   ├── my_slam_launch.py                # Sadece SLAM
│   │   │   ├── my_nav2_launch.py                # Navigasyon (harita ile)
│   │   │   └── my_robot_gazebo.launch.xml       # Gazebo simülasyonu
│   │   ├── config/
│   │   │   ├── explore_params.yaml              # 🔍 Keşif parametreleri
│   │   │   ├── nav2_params.yaml                 # 🧭 Nav2 parametreleri
│   │   │   ├── mapper_params_online_async.yaml  # 🗺️ SLAM parametreleri
│   │   │   ├── my_controller.yaml               # 🎮 Controller ayarları
│   │   │   └── twist_mux.yaml                    # Twist mux ayarları
│   │   ├── models/                              # Gazebo modelleri
│   │   ├── worlds/                              # Gazebo dünya dosyaları
│   │   └── rviz/                                # RViz konfigürasyonları
│   ├── my_robot_description/                    # Robot URDF/XACRO dosyaları
│   │   └── urdf/
│   │       ├── main.xacro
│   │       ├── my_robot.xacro
│   │       ├── ros2_control.xacro
│   │       └── ...
│   ├── m-explore-ros2/                          # Otonom keşif paketi
│   │   ├── explore/                            # Frontier-based exploration
│   │   └── map_merge/                          # Harita birleştirme
│   └── lidar_py_pkg/                           # LIDAR veri işleme
├── maps/                                       # Kaydedilmiş haritalar
│   ├── ilk_map.pgm
│   ├── ilk_map.yaml
│   └── ...
└── README.md
```

## ⚙️ Konfigürasyon

### Keşif Parametreleri (`explore_params.yaml`)

Otonom keşif davranışını kontrol eden kritik parametreler:

```yaml
explore:
  ros__parameters:
    # Keşif stratejisi
    planner_frequency: 0.1      # Hedef belirleme sıklığı (Hz)
    progress_timeout: 120.0      # Hedef timeout (saniye)
    potential_scale: 0.1         # Yakınlık tercihi (düşük = uzak alanları da keşfet)
    gain_scale: 30.0            # Bilgi kazancı tercihi (yüksek = büyük alanları tercih et)
    min_frontier_size: 0.2      # Minimum frontier boyutu (metre)
    explore_radius: 20.0         # Keşif yarıçapı (metre)
    return_to_init: false       # Keşif bitince başlangıca dön
```

### Nav2 Parametreleri (`nav2_params.yaml`)

Navigasyon ve güvenlik ayarları:

- **Inflation Radius**: 0.7m (duvarlardan mesafe)
- **Cost Scaling Factor**: 6.0 (obstacle cost)
- **Max Velocity**: 0.4 m/s
- **Planner Tolerance**: 5.0m

### SLAM Parametreleri (`mapper_params_online_async.yaml`)

SLAM harita oluşturma ayarları:

- **Mode**: `mapping` (otonom keşif için)
- **Resolution**: 0.05m
- **Map Update Rate**: 5.0 Hz

## 🎯 Launch Dosyaları

| Launch Dosyası | Açıklama |
|---------------|----------|
| `autonomous_exploration.launch.py` | ⭐ **Otonom SLAM keşif** - Robot kendi kendine harita oluşturur ve keşfeder |
| `slam_navigation.launch.py` | SLAM + Navigasyon (manuel kontrol) |
| `my_slam_launch.py` | Sadece SLAM harita oluşturma |
| `my_nav2_launch.py` | Önceden oluşturulmuş harita ile navigasyon |
| `my_robot_gazebo.launch.xml` | Sadece Gazebo simülasyonu |

## 🔧 Transform Frame Yapısı

```
map (SLAM tarafından yayınlanır)
  └── odom (AMCL/odometry tarafından yayınlanır)
      └── base_footprint (ROS2 Control tarafından yayınlanır)
          └── base_link
              ├── lidar_link
              ├── camera_link
              └── ...
```

## 🐛 Sorun Giderme

### Robot "out of costmap bounds" Hatası

- `explore_node`'un başlatılması için yeterli süre bekleyin (18 saniye)
- SLAM'ın harita oluşturduğundan emin olun: `ros2 topic echo /map`
- `costmap_topic: /map` parametresinin doğru olduğunu kontrol edin

### Robot Takılıyor veya Hedeflere Gidemiyor

- `nav2_params.yaml`'da `tolerance` değerini artırın (örn: 5.0m)
- `explore_params.yaml`'da `progress_timeout` değerini artırın
- `inflation_radius` ve `cost_scaling_factor` değerlerini ayarlayın

### Harita Oluşturulmuyor

- LIDAR verisinin geldiğini kontrol edin: `ros2 topic echo /lidar/out`
- SLAM node'unun çalıştığını kontrol edin: `ros2 node list | grep slam`
- `mapper_params_online_async.yaml`'da `mode: mapping` olduğundan emin olun

### TF Hataları

- `use_sim_time: true` parametresinin tüm node'larda ayarlandığından emin olun
- `transform_tolerance` değerlerini artırabilirsiniz (nav2_params.yaml)
- TF tree'yi kontrol edin: `ros2 run tf2_tools view_frames`

## 📸 Ekran Görüntüleri

*(Ekran görüntüleri eklenecek)*

## 🤝 Katkıda Bulunma

Pull request'ler memnuniyetle karşılanır! Büyük değişiklikler için lütfen önce bir issue açarak neyi değiştirmek istediğinizi tartışın.

### Katkıda Bulunma Adımları

1. Fork edin
2. Feature branch oluşturun (`git checkout -b feature/AmazingFeature`)
3. Değişikliklerinizi commit edin (`git commit -m 'Add some AmazingFeature'`)
4. Branch'inizi push edin (`git push origin feature/AmazingFeature`)
5. Pull Request açın

## 📝 Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylar için `LICENSE` dosyasına bakın.

## 👤 Yazar

**EnesKotay**

- GitHub: [@EnesKotay](https://github.com/EnesKotay)

## 🙏 Teşekkürler

- [ROS2 Navigation2](https://navigation.ros.org/) - Navigation framework
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM implementation
- [m-explore-ros2](https://github.com/M0RF3US/m-explore-ros2) - Frontier-based exploration
- [ROS2 Control](https://control.ros.org/) - Robot control framework
- [Gazebo](https://gazebosim.org/) - Physics simulation

## 📚 Referanslar

- [ROS2 Navigation2 Documentation](https://navigation.ros.org/)
- [slam_toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Control Documentation](https://control.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Frontier-Based Exploration](https://github.com/M0RF3US/m-explore-ros2)

## 🎯 Gelecek Özellikler

- [ ] Multi-robot exploration
- [ ] Dynamic obstacle avoidance improvements
- [ ] Map saving/loading automation
- [ ] Performance optimizations
- [ ] Real robot deployment support

---

⭐ Bu projeyi beğendiyseniz yıldız vermeyi unutmayın!

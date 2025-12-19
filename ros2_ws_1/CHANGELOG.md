# Changelog

Tüm önemli değişiklikler bu dosyada dokümante edilecektir.

Format [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) baz alınarak oluşturulmuştur,
ve bu proje [Semantic Versioning](https://semver.org/spec/v2.0.0.html) kullanmaktadır.

## [Unreleased]

### Added
- Otonom SLAM keşif özelliği (`autonomous_exploration.launch.py`)
- `m-explore-ros2` paketi entegrasyonu
- Frontier-based exploration algoritması
- Sıralı node başlatma (TimerAction) ile güvenilir başlatma
- Gelişmiş keşif parametreleri (`explore_params.yaml`)
- Nav2 optimizasyonları (inflation, cost scaling, velocity)
- Map dosyalarını `maps/` klasörüne taşıma

### Changed
- `explore_params.yaml`: `/map` topic'ini kullanacak şekilde güncellendi
- `nav2_params.yaml`: Daha güvenli navigasyon için parametreler optimize edildi
- Launch dosyaları: Sıralı başlatma eklendi
- `mapper_params_online_async.yaml`: Mapping moduna geçirildi

### Fixed
- "Robot out of costmap bounds" hatası düzeltildi
- Explore node başlatma zamanlaması optimize edildi
- TF synchronization sorunları giderildi

## [1.0.0] - 2025-12-19

### Added
- İlk stabil sürüm
- ROS2 Navigation2 entegrasyonu
- SLAM Toolbox desteği
- Gazebo simülasyonu
- AMCL lokalizasyonu
- Nav2 navigasyon stack'i
- RViz2 görselleştirme
- Robot URDF/XACRO tanımları
- ROS2 Control entegrasyonu
- LIDAR ve kamera sensörleri
- Temel launch dosyaları
- Konfigürasyon dosyaları

### Changed
- README.md güncellendi ve genişletildi
- Proje yapısı düzenlendi

---

[Unreleased]: https://github.com/EnesKotay/my_ros2_project/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/EnesKotay/my_ros2_project/releases/tag/v1.0.0


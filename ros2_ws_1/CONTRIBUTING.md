# Katkıda Bulunma Rehberi

Bu projeye katkıda bulunmak istediğiniz için teşekkürler! 🎉

## 🚀 Başlangıç

1. Repository'yi fork edin
2. Local'inize clone edin:
   ```bash
   git clone https://github.com/YOUR_USERNAME/my_ros2_project.git
   cd my_ros2_project/ros2_ws_1
   ```
3. Remote'u ayarlayın:
   ```bash
   git remote add upstream https://github.com/EnesKotay/my_ros2_project.git
   ```

## 🔀 Çalışma Akışı

1. **Branch oluşturun**
   ```bash
   git checkout -b feature/your-feature-name
   # veya
   git checkout -b fix/your-bug-fix
   ```

2. **Değişikliklerinizi yapın**
   - Kod yazarken ROS2 best practices'i takip edin
   - Yorum satırları ekleyin (özellikle karmaşık algoritmalar için)
   - Kod formatını koruyun

3. **Test edin**
   ```bash
   colcon build
   source install/setup.bash
   # Launch dosyanızı test edin
   ```

4. **Commit edin**
   ```bash
   git add .
   git commit -m "feat: Add amazing feature"
   # veya
   git commit -m "fix: Fix navigation bug"
   ```

5. **Push edin**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Pull Request oluşturun**
   - GitHub'da Pull Request açın
   - Değişikliklerinizi açıklayın
   - İlgili issue'ları referans edin

## 📝 Commit Mesajları

Commit mesajlarınızı şu formatlarda yazın:

- `feat: Yeni özellik eklendi`
- `fix: Bug düzeltildi`
- `docs: Dokümantasyon güncellendi`
- `style: Kod formatı düzeltildi`
- `refactor: Kod refactor edildi`
- `test: Test eklendi`
- `chore: Build/config değişiklikleri`

## 🧪 Test Etme

Pull request göndermeden önce:

- [ ] Kod derleniyor (`colcon build`)
- [ ] Launch dosyası çalışıyor
- [ ] Yeni özellik test edildi
- [ ] Mevcut özellikler bozulmadı

## 📋 Kod Standartları

- **Python**: PEP 8 standartlarını takip edin
- **C++**: ROS2 C++ style guide'ı takip edin
- **YAML**: 2 space indentation kullanın
- **Launch dosyaları**: Açıklayıcı yorumlar ekleyin

## 🐛 Bug Raporlama

Bug bulduysanız:

1. Issue açın
2. Açıklayıcı başlık kullanın
3. Adımları detaylıca açıklayın
4. Beklenen ve gerçek davranışı karşılaştırın
5. Log dosyalarını ekleyin (varsa)

## 💡 Özellik Önerileri

Yeni özellik önerirken:

1. Issue açın ve "Feature Request" etiketi ekleyin
2. Özelliğin neden gerekli olduğunu açıklayın
3. Kullanım senaryosunu detaylandırın
4. Alternatif çözümleri değerlendirin

## ❓ Sorular

Sorularınız için:
- Issue açabilirsiniz
- Discussions bölümünü kullanabilirsiniz

## 📄 Lisans

Katkıda bulunarak, katkılarınızın MIT lisansı altında lisanslanacağını kabul etmiş olursunuz.

---

Teşekkürler! 🙏


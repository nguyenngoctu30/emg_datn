# 🦾 Ứng Dụng Cảm Biến EMG Vào Điều Khiển Cánh Tay Máy Từ Xa

![Badge Status](https://img.shields.io/badge/Status-Active-green) ![Badge License](https://img.shields.io/badge/License-MIT-blue) ![Badge Version](https://img.shields.io/badge/Version-1.0.0-orange)

---

## 📋 Mục Lục

- [Giới Thiệu](#-giới-thiệu)
- [Tính Năng Chính](#-tính-năng-chính)
- [Kiến Trúc Hệ Thống](#-kiến-trúc-hệ-thống)
- [Yêu Cầu Hệ Thống](#-yêu-cầu-hệ-thống)
- [Cài Đặt](#-cài-đặt)
- [Hướng Dẫn Sử Dụng](#-hướng-dẫn-sử-dụng)
- [Cấu Trúc Dự Án](#-cấu-trúc-dự-án)
- [Công Nghệ Sử Dụng](#-công-nghệ-sử-dụng)
- [Những Tính Năng Nổi Bật](#-những-tính-năng-nổi-bật)
- [Hỗ Trợ & Liên Hệ](#-hỗ-trợ--liên-hệ)

---

## 🎯 Giới Thiệu

**EMG (Electromyography) Control System** là một hệ thống điều khiển cánh tay máy tiên tiến sử dụng:
- **Cảm biến EMG** để phát hiện tín hiệu điện cơ
- **Web Interface** để giám sát và điều khiển từ xa
- **Arduino/ESP32** xử lý tín hiệu thời gian thực
- **Giao diện 3D** để hiển thị trạng thái cánh tay máy

Hệ thống này cho phép người dùng điều khiển cánh tay máy bằng các tín hiệu cơ (các cơ của tay/cánh tay), mang lại trải nghiệm tương tác trực quan và hiệu quả.

---

## ✨ Tính Năng Chính

| Tính Năng | Mô Tả |
|-----------|-------|
| 📊 **Giám Sát Tín Hiệu** | Hiển thị tín hiệu EMG theo thời gian thực bằng biểu đồ |
| 🎮 **Điều Khiển Từ Xa** | Điều khiển cánh tay máy qua giao diện web |
| 📱 **Responsive Design** | Hỗ trợ trên các thiết bị: máy tính, tablet, điện thoại |
| 🔄 **Cập Nhật OTA** | Cập nhật firmware từ xa không cần kết nối USB |
| 💾 **Xuất Dữ Liệu** | Tải dữ liệu cảm biến dưới dạng file Excel |
| 📈 **Phân Tích Tín Hiệu** | Hiển thị các thông số: RMS, Frequency, Amplitude |
| 🌐 **Kết Nối MQTT** | Giao tiếp với máy chủ MQTT cho các ứng dụng phân tán |
| 🔌 **Điều Khiển Servo** | Điều khiển chuyển động của các servo motor chính xác |

---

## 🏗️ Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────┐
│                   Web Browser                            │
│            (index.html, control.html, etc.)             │
└────────────────────┬────────────────────────────────────┘
                     │ HTTP/MQTT
┌────────────────────▼────────────────────────────────────┐
│              ESP32/Arduino                              │
│         (emg.ino, servo.ino)                            │
│  - EMG Signal Processing                               │
│  - Servo Motor Control                                 │
│  - WiFi/MQTT Communication                            │
└────────────────────┬────────────────────────────────────┘
                     │ Analog Signal
┌────────────────────▼────────────────────────────────────┐
│          EMG Sensors + Servo Motors                     │
│        (Bộ cảm biến + Cánh tay máy)                    │
└─────────────────────────────────────────────────────────┘
```

---

## 📦 Yêu Cầu Hệ Thống

### Phần Cứng
- **Bộ vi điều khiển**: ESP32 hoặc Arduino
- **Cảm biến EMG**: 2 cảm biến (Pin 0, 1)
- **Servo Motor**: Các servo để điều khiển cánh tay
- **LED**: 1 LED RGB (Pin 2) - tùy chọn
- **WiFi Module**: Tích hợp sẵn trong ESP32

### Phần Mềm
- **Arduino IDE**: Để lập trình microcontroller
- **Thư viện**: EMGFilters, PubSubClient, FastLED
- **Web Browser**: Chrome, Firefox, Edge, Safari (phiên bản mới)
- **Server**: MQTT Broker (tùy chọn)

### Thư Viện JavaScript
- Chart.js - Vẽ biểu đồ
- XLSX - Xuất Excel
- MQTT.js - Giao tiếp MQTT
- Model-Viewer - Hiển thị 3D

---

## 🚀 Cài Đặt

### 1️⃣ **Clone Repository**
```bash
git clone <repository-url>
cd web_emg-main
```

### 2️⃣ **Cài Đặt Arduino IDE**
- Tải xuống [Arduino IDE](https://www.arduino.cc/en/software)
- Cài đặt bộ hỗ trợ cho ESP32 hoặc Arduino board của bạn

### 3️⃣ **Cài Đặt Thư Viện Arduino**
Vào Arduino IDE → Sketch → Include Library → Manage Libraries, tìm và cài đặt:
- **EMGFilters**
- **PubSubClient**
- **FastLED**
- **HTTPClient** (tích hợp sẵn)

### 4️⃣ **Cấu Hình WiFi**
Trong file `emg.ino`, cập nhật WiFi credentials:
```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_SERVER";
```

### 5️⃣ **Upload Firmware**
- Kết nối board via USB
- Chọn đúng board type và port
- Click Upload (Ctrl + U)

### 6️⃣ **Chạy Web Interface**
- Mở file `index.html` trong trình duyệt
- Hoặc dùng web server (khuyến nghị):
```bash
python -m http.server 8000
# Sau đó truy cập: http://localhost:8000
```

---

## 📖 Hướng Dẫn Sử Dụng

### 🏠 Trang Chủ (index.html)
- Xem tín hiệu EMG thời gian thực
- Hiển thị biểu đồ signal + FFT
- Xem mô hình 3D cánh tay
- Theo dõi các thông số cảm biến

### 🎮 Điều Khiển (control.html)
- Điều khiển các servo motor
- Mở/Đóng cánh tay bằng nút lệnh
- Điều chỉnh tốc độ chuyển động
- Lưu trữ các cấu hình yêu thích

### ℹ️ Giới Thiệu (gioithieu.html)
- Thông tin về hệ thống
- Hướng dẫn sử dụng chi tiết
- Thông tin kỹ thuật

### 📰 Tin Tức (tintuc.html)
- Cập nhật và thông báo về dự án

### 🔄 OTA Update (ota.html)
- Cập nhật firmware từ xa
- Không cần kết nối USB
- Theo dõi tiến trình cập nhật

---

## 📁 Cấu Trúc Dự Án

```
web_emg-main/
├── 📄 index.html              # Trang chủ - Giám sát tín hiệu
├── 📄 control.html            # Trang điều khiển cánh tay
├── 📄 gioithieu.html          # Trang giới thiệu
├── 📄 tintuc.html             # Trang tin tức
├── 📄 ota.html                # Trang cập nhật firmware
├── 📄 progress.html           # Trang báo cáo tiến độ
│
├── 🔧 emg.ino                 # Code chính ESP32
├── 🔧 servo.ino               # Code điều khiển servo (nếu cần)
│
├── 💻 script.js               # JavaScript chính (logic trang chủ)
├── 💻 control.js              # Logic điều khiển cánh tay
├── 💻 common.js               # Hàm tiện ích chung
├── 💻 ota.js                  # Logic cập nhật OTA
│
├── 🎨 styles.css              # CSS trang chủ
├── 🎨 shared.css              # CSS chung
├── 🎨 share.css               # CSS bổ sung
├── 🎨 ota.css                 # CSS trang OTA
│
├── 🖼️ 3D Models:
│   ├── arm.png                # Ảnh cánh tay
│   ├── Canhtayv3.glb          # Mô hình 3D cánh tay
│   └── box2.glb               # Mô hình 3D phụ
│
├── 🖼️ Logos & Icons:
│   ├── logo.png               # Logo chính
│   ├── logo_fet.png           # Logo FET
│   ├── image.png              # Ảnh
│   ├── emg_pic.png            # Ảnh EMG
│   ├── emg_signal.png         # Ảnh signal EMG
│   ├── emg_value.png          # Ảnh giá trị EMG
│   ├── open_hand.png          # Ảnh tay mở
│   └── closed_hand.png        # Ảnh tay đóng
│
└── 📋 README.md               # File này
```

---

## 🛠️ Công Nghệ Sử Dụng

### Backend/Firmware
| Công Nghệ | Mục Đích |
|-----------|---------|
| Arduino/ESP32 | Microcontroller chính |
| EMGFilters | Xử lý tín hiệu EMG |
| PubSubClient | Giao tiếp MQTT |
| FastLED | Điều khiển LED RGB |
| WiFi | Kết nối mạng không dây |

### Frontend
| Công Nghệ | Mục Đích |
|-----------|---------|
| HTML5 | Cấu trúc giao diện |
| CSS3 + Tailwind | Styling và responsive |
| JavaScript | Logic ứng dụng |
| Chart.js | Vẽ biểu đồ |
| Model-Viewer | Hiển thị 3D |
| XLSX | Xuất Excel |
| MQTT.js | Giao tiếp MQTT |

---

## ⭐ Những Tính Năng Nổi Bật

### 🔬 Xử Lý Tín Hiệu Chuyên Nghiệp
- Lọc nhiễu tần số 50Hz
- Hiệu chuẩn baseline tự động
- Phân tích FFT thời gian thực
- Tính toán RMS và Frequency

### 📊 Trực Quan Hoá Dữ Liệu
- Biểu đồ động theo thời gian thực
- Hiển thị FFT spectrum
- Mô hình 3D tương tác
- Giao diện responsive

### 🔐 Tính Ổn Định & An Toàn
- Kết nối WiFi mạnh
- MQTT cho communication đáng tin cậy
- OTA update an toàn
- Xử lý lỗi toàn diện

### ⚡ Hiệu Suất Cao
- Xử lý tín hiệu 1000Hz
- Cập nhật giao diện mượt mà
- Tối ưu hóa bandwidth
- Hỗ trợ nhiều client

---

## 🔬 Các Phương Pháp Xử Lý & Lọc Dữ Liệu

### 📊 K-Means Clustering (Phân Cụm K-Means)

K-Means được sử dụng để phân loại các mẫu tín hiệu EMG thành các nhóm khác nhau, giúp nhận diện các cơ đang hoạt động.

#### Công Thức:

**1. Gán điểm dữ liệu đến cluster gần nhất:**
$$x_i \in C_k \text{ nếu } ||x_i - \mu_k||^2 < ||x_i - \mu_j||^2 \text{ với } j \neq k$$

Trong đó:
- $x_i$ = điểm dữ liệu thứ i
- $C_k$ = cluster k
- $\mu_k$ = tâm (centroid) của cluster k

**2. Cập nhật tâm cluster:**
$$\mu_k^{(t+1)} = \frac{1}{|C_k|} \sum_{x_i \in C_k} x_i$$

**3. Hàm mất mát (Loss Function):**
$$J = \sum_{k=1}^{K} \sum_{x_i \in C_k} ||x_i - \mu_k||^2$$

#### Ứng Dụng:
- Phân loại các mẫu hoạt động cơ khác nhau
- Nhận diện các cử chỉ (gesture recognition)
- Phân tách tín hiệu từ nhiều cảm biến

#### Ví Dụ Code (Python):
```python
from sklearn.cluster import KMeans
import numpy as np

# Dữ liệu EMG đã xử lý
emg_data = np.array([[...], [...], ...])

# Khởi tạo K-Means với 3 clusters
kmeans = KMeans(n_clusters=3, random_state=42, n_init=10)
clusters = kmeans.fit_predict(emg_data)

# In kết quả phân cụm
print("Tâm clusters:", kmeans.cluster_centers_)
print("Nhãn clusters:", clusters)
```

---

### 📈 3 Sigma Rule (Quy Tắc 3 Sigma)

Quy tắc 3 Sigma được dùng để phát hiện và loại bỏ các giá trị ngoại lệ (outliers) trong tín hiệu EMG, giúp cải thiện chất lượng dữ liệu.

#### Nguyên Lý:
Trong phân phối chuẩn:
- **68.27%** dữ liệu nằm trong ±1σ
- **95.45%** dữ liệu nằm trong ±2σ
- **99.73%** dữ liệu nằm trong ±3σ

#### Công Thức:

**1. Tính trung bình (Mean):**
$$\mu = \frac{1}{n} \sum_{i=1}^{n} x_i$$

**2. Tính độ lệch chuẩn (Standard Deviation):**
$$\sigma = \sqrt{\frac{1}{n} \sum_{i=1}^{n} (x_i - \mu)^2}$$

**3. Xác định các giá trị ngoài 3 Sigma:**
$$\text{Outlier nếu: } |x_i - \mu| > 3\sigma$$

**4. Tính khoảng tin cậy 3 Sigma:**
$$[\mu - 3\sigma, \mu + 3\sigma]$$

#### Ứng Dụng:
- Loại bỏ nhiễu và spike không mong muốn
- Phát hiện các bất thường trong tín hiệu
- Cải thiện độ chính xác của phân loại cử chỉ

#### Ví Dụ Code (Python):
```python
import numpy as np

# Tín hiệu EMG thô
emg_signal = np.array([...])

# Tính trung bình và độ lệch chuẩn
mean = np.mean(emg_signal)
std = np.std(emg_signal)

# Xác định khoảng 3 Sigma
lower_bound = mean - 3 * std
upper_bound = mean + 3 * std

# Lọc tín hiệu
filtered_signal = emg_signal[
    (emg_signal >= lower_bound) & 
    (emg_signal <= upper_bound)
]

# Xác định outliers
outliers = emg_signal[
    (emg_signal < lower_bound) | 
    (emg_signal > upper_bound)
]

print(f"Trung bình: {mean:.2f}")
print(f"Độ lệch chuẩn: {std:.2f}")
print(f"Khoảng 3 Sigma: [{lower_bound:.2f}, {upper_bound:.2f}]")
print(f"Số outliers phát hiện: {len(outliers)}")
```

---

### 🧮 Các Công Thức Lọc Giá Trị Khác

#### 1. **Moving Average Filter (Bộ Lọc Trung Bình Động)**
$$y_n = \frac{1}{N} \sum_{i=0}^{N-1} x_{n-i}$$

- **Ưu điểm**: Loại bỏ nhiễu high frequency
- **Nhược điểm**: Làm mất chi tiết tín hiệu

#### 2. **Exponential Moving Average (EMA)**
$$y_n = \alpha x_n + (1 - \alpha) y_{n-1}$$

Trong đó: $\alpha = \frac{2}{N+1}$ (hệ số mượt hoá)
- **Ưu điểm**: Nặng giá trị hiện tại, mượt mà hơn
- **Ứng dụng**: Giám sát tín hiệu thời gian thực

#### 3. **Butterworth Low-Pass Filter**
$$H(f) = \frac{1}{\sqrt{1 + \left(\frac{f}{f_c}\right)^{2n}}}$$

Trong đó:
- $f_c$ = tần số cắt (cutoff frequency)
- $n$ = bậc của filter
- **Ứng dụng**: Lọc tần số cao trong tín hiệu EMG

#### 4. **Notch Filter (Lọc Khước)**
$$H(f) = \frac{f^2 - \omega_0^2 + jB\omega}{f^2 - \omega_0^2 + jBf}$$

- **Mục đích**: Loại bỏ nhiễu tần số 50Hz (hoặc 60Hz)
- **Ứng dụng**: Lọc nhiễu từ dòng điện AC

#### 5. **Z-Score Normalization (Chuẩn Hóa Z-Score)**
$$z = \frac{x - \mu}{\sigma}$$

- **Mục đích**: Chuẩn hóa dữ liệu trong khoảng [-3, 3]
- **Ứng dụng**: Chuẩn bị dữ liệu cho machine learning

---

### 💡 Ví Dụ Kết Hợp: Pipeline Xử Lý Tín Hiệu EMG

```python
import numpy as np
from scipy import signal
from sklearn.preprocessing import StandardScaler

# 1. Đọc tín hiệu thô
raw_emg = load_emg_signal()

# 2. Áp dụng Notch Filter (loại bỏ 50Hz)
b, a = signal.iirnotch(50, 30, fs=1000)
notch_filtered = signal.filtfilt(b, a, raw_emg)

# 3. Áp dụng Butterworth Low-Pass Filter (100Hz)
b, a = signal.butter(4, 100, fs=1000, btype='low')
butterworth_filtered = signal.filtfilt(b, a, notch_filtered)

# 4. Áp dụng 3 Sigma Rule
mean = np.mean(butterworth_filtered)
std = np.std(butterworth_filtered)
sigma_filtered = butterworth_filtered[
    np.abs(butterworth_filtered - mean) <= 3 * std
]

# 5. Chuẩn hóa Z-Score
scaler = StandardScaler()
normalized = scaler.fit_transform(sigma_filtered.reshape(-1, 1))

# 6. Áp dụng K-Means để phân loại
from sklearn.cluster import KMeans
kmeans = KMeans(n_clusters=3, random_state=42)
gesture_classes = kmeans.fit_predict(normalized)

print("✅ Pipeline xử lý hoàn tất!")
print(f"Số mẫu sau lọc: {len(sigma_filtered)}")
print(f"Cử chỉ phân loại: {np.unique(gesture_classes)}")
```

---

## 🤝 Hỗ Trợ & Liên Hệ

Nếu bạn gặp vấn đề:

1. **Kiểm tra lại**:
   - Kết nối WiFi có đúng không?
   - Cảm biến có được kết nối chính xác không?
   - Board có được nạp code thành công không?

2. **Xem logs**:
   - Mở Serial Monitor để xem debug message
   - Kiểm tra console browser (F12) để xem JavaScript errors

3. **Liên hệ với tác giả**:
   - 📧 Email: [nguyenngoctu1630@gmail.com](mailto:nguyenngoctu1630@gmail.com)
   - 🐱 GitHub: [@nguyenngoctu30](https://github.com/nguyenngoctu30)
   - 💬 Web: [Web EMG Project](https://emgdatn.vercel.app/)

---

## 📝 License

Dự án này được cấp phép dưới [MIT License](LICENSE)

---

## 🙏 Cảm Ơn

Cảm ơn bạn đã sử dụng hệ thống EMG Control System. 
Nếu thấy hữu ích, hãy ⭐ star dự án này!

**Happy Controlling!** 🚀

---

<div align="center">

**Phiên bản: 1.0.0** | **Cập nhật: Tháng 1, 2026**

Made with ❤️ for Robotics & IoT Enthusiasts

</div>


# Pengumpulan Data dari Berbagai Sensor

## Langkah-langkah

1. **Git Pull atau Clone**  
   Sebelum memulai, pastikan untuk memperbarui repository dengan menjalankan:
   ```bash
   git pull origin main
   ```
   atau jika belum memiliki repository, gunakan:
   ```bash
   git clone <URL_repository>
   ```

2. **Masuk ke Direktori Catkin Workspace**  
   Navigasi ke direktori Catkin Workspace:
   ```bash
   cd ~/catkin_ws
   ```

3. **Hapus Folder `devel` dan `build`**  
   Untuk memastikan tidak ada sisa build yang mengganggu, hapus folder `devel` dan `build`:
   ```bash
   sudo rm -rf devel build
   ```

4. **Jalankan Docker Compose**  
   Untuk menyiapkan lingkungan, jalankan:
   ```bash
   sudo docker compose up
   ```

5. **Di Dalam Docker, Jalankan Catkin Make**  
   Setelah Docker berjalan, masuk ke direktori di atas dan jalankan:
   ```bash
   cd .. && catkin_make
   ```

6. **Jalankan Launch File**  
   Setelah build selesai, jalankan file launch:
   ```bash
   roslaunch gather_data gather_data.launch
   ```

## Hal yang Harus Diperhatikan

Sebelum menyalakan Jetson, pastikan untuk menyambungkan seluruh USB ke portnya masing-masing. Pastikan kabel kamera dan GPS terhubung langsung ke Jetson.

Jika terjadi error saat menjalankan launch, lakukan debugging dengan mengikuti langkah berikut:

1. **Navigasi ke Direktori Utilitas**  
   ```bash
   cd /app/src/gather_data/src/utilx
   ```

2. **Jalankan Python File Sesuai Dengan Device**  
   Misalkan untuk debugging kamera, jalankan:
   ```bash
   python3 camera.py
   ```

   Untuk low-level sensor, gunakan `rosserial` untuk debugging dengan perintah berikut:
   ```bash
   rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1
   ```
   (Port tergantung pada low-level sensor yang disambungkan).

### Keterangan Tambahan
- IMU menggunakan port: `/dev/ttyUSB0`
- GPS (Kumar/u-blox) menggunakan port: `/dev/ttyACM0`
- Low-level sensor lainnya menggunakan port: `/dev/ttyACM1` hingga `/dev/ttyACM3`

Jika ada port yang tidak terbaca, Anda dapat menjalankan `docker compose up` ulang atau merestart Jetson.

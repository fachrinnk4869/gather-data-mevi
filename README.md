# Pengumpulan Data dari Berbagai Sensor

## Langkah-langkah

1. **Clone Repository**

   Sebelum memulai, pastikan untuk memperbarui repository dengan menjalankan:
   ```bash
   git pull origin main
   ```
   atau jika belum memiliki repository, gunakan:
   ```bash
   git clone <URL_repository>
   ```

2. **Masuk ke Workspace**
   ```bash
   cd ~/catkin_ws
   ```

3. **Bersihkan Build**
   ```bash
   sudo rm -rf devel build
   ```

4. **Jalankan Docker**
   ```bash
   sudo docker compose up
   ```

5. **Masuk ke Terminal Docker**
   - Buka tab terminal baru dan jalankan:
   ```bash
   sudo docker exec -it zed_jetson_camera_2 /bin/bash
   ```
   - Pastikan menjalankan perintah ini jika ingin menjalankan perintah ROS apapun.

6. **Build Workspace di Dalam Docker**
   ```bash
   cd ..
   catkin_make
   ```

7. **Jalankan Launch File**
   ```bash
   roslaunch gather_data gather_data.launch
   ```

## Hal yang Harus Diperhatikan

Sebelum menyalakan Jetson, pastikan untuk menyambungkan seluruh USB ke portnya masing-masing. Pastikan kabel kamera dan GPS terhubung langsung ke Jetson.

Jika masih ada error saat menjalankan launch, lakukan debugging dengan cara berikut:

1. Masuk ke direktori utilitas di terminal docker:
   ```bash
   cd /app/src/gather_data/src/utilx
   ```
   - pastikan sudah masuk terlebih dulu ke terminal docker

2. Jalankan file Python masing-masing dengan device yang ingin dilakukan debugging. Misalnya, untuk kamera:
   ```bash
   python3 camera.py
   ```
   - pastikan sudah masuk terlebih dulu ke terminal docker

3. Untuk sensor low-level, gunakan `rosserial` untuk debugging dengan cara:
   ```bash
   rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1
   ```
   (Port tergantung pada port low-level yang disambungkan.)
   - pastikan sudah masuk terlebih dulu ke terminal docker
### Keterangan Tambahan

- IMU menggunakan port: `/dev/ttyUSB0`
- GPS (Kumar/UBlox) menggunakan port: `/dev/ttyACM0`
- Sensor low-level lainnya menggunakan port: `/dev/ttyACM1` hingga `/dev/ttyACM3`

Jika ada port yang tidak terbaca, coba jalankan `docker compose up` ulang atau restart Jetson.
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Float32.h>

Adafruit_MCP4725 dac;

// Faktor skala untuk konversi nilai digital ke tegangan aktual
const float faktor_skala = 3.3 / 4095.0;

// Konstanta untuk menghitung kecepatan
const float wheel_diameter = 0.36;  // Diameter roda dalam meter
const float pulses_per_revolution = 400.0;  // Jumlah pulsa per revolusi dari encoder

// Pin untuk encoder optik
const int encoderPin1A = 3;
const int encoderPin1B = 2;
const int encoderPin2A = 4;
const int encoderPin2B = 5;

// Membuat objek encoder
Encoder myEnc1(encoderPin1A, encoderPin1B);
Encoder myEnc2(encoderPin2A, encoderPin2B);

// Variabel untuk menyimpan nilai posisi encoder sebelumnya
long oldPosition1 = 0;
long oldPosition2 = 0;
unsigned long lastTime1 = 0;
unsigned long lastTime2 = 0;

// ROS nodehandle dan publisher
ros::NodeHandle nh;

std_msgs::Float32 enc1_msg;
ros::Publisher enc1_pub("encoder1_value", &enc1_msg);

std_msgs::Float32 enc2_msg;
ros::Publisher enc2_pub("encoder2_value", &enc2_msg);

// Callback untuk menerima nilai DAC dari topik "dac_value"
void dacCallback(const std_msgs::Float32& msg) {
  uint16_t dac_value = (uint16_t)msg.data;  // Ambil nilai DAC dari pesan
  dac.setVoltage(dac_value, false);         // Set DAC dengan nilai yang diterima
  Serial.print("Set DAC value: ");
  Serial.println(dac_value);
}

// Subscriber untuk mendengarkan topik "dac_value"
ros::Subscriber<std_msgs::Float32> dac_sub("dac_value", &dacCallback);

void setup() {
  Serial.begin(9600);
  dac.begin(0x60);  // Alamat I2C default untuk MCP4725
  Serial.println("MCP4725 DAC");

  // Inisialisasi node ROS dan publisher
  nh.initNode();
  nh.advertise(enc1_pub);
  nh.advertise(enc2_pub);

  // Inisialisasi subscriber untuk DAC
  nh.subscribe(dac_sub);
}

void loop() {
  // Baca posisi encoder 1
  long newPosition1 = myEnc1.read();
  if (newPosition1 != oldPosition1) {
    unsigned long currentTime1 = millis();
    long pulseDifference1 = newPosition1 - oldPosition1;
    unsigned long timeDifference1 = currentTime1 - lastTime1;

    // Hitung kecepatan (m/s) untuk encoder 1
    float distance1 = (pulseDifference1 / pulses_per_revolution) * (PI * wheel_diameter);
    float velocity1 = distance1 / (timeDifference1 / 1000.0); // m/s

    // Publikasikan nilai encoder 1
    enc1_msg.data = velocity1;
    enc1_pub.publish(&enc1_msg);

    oldPosition1 = newPosition1;
    lastTime1 = currentTime1;
  }
  else{
    enc1_msg.data = 0.0;
    enc1_pub.publish(&enc1_msg);
  }

  // Baca posisi encoder 2
  long newPosition2 = myEnc2.read();
  if (newPosition2 != oldPosition2) {
    unsigned long currentTime2 = millis();
    long pulseDifference2 = newPosition2 - oldPosition2;
    unsigned long timeDifference2 = currentTime2 - lastTime2;

    // Hitung kecepatan (m/s) untuk encoder 2
    float distance2 = (pulseDifference2 / pulses_per_revolution) * (PI * wheel_diameter);
    float velocity2 = distance2 / (timeDifference2 / 1000.0); // m/s

    // Publikasikan nilai encoder 2
    enc2_msg.data = velocity2;
    enc2_pub.publish(&enc2_msg);

    oldPosition2 = newPosition2;
    lastTime2 = currentTime2;
  }
  else{
    enc2_msg.data = 0.0;
    enc2_pub.publish(&enc2_msg);
  }

  nh.spinOnce(); // Memproses callback ROS
  delay(10);    // Delay kecil
}

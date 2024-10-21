#include <Adafruit_MCP4725.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32.h>

Adafruit_MCP4725 dac;
// PID Variables
double Setpoint = 0.0;
double Input, Output,Input2,mvo;
double wheel_diameter = 0.36;
// PID Tuning Parameters
double Kp = 10.0, Ki = 0.5, Kd = 0.6;
//2 1 1;


// Create PID Object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ROS nodehandle and subscribers
ros::NodeHandle nh;

std_msgs::Float32 out_msg;
ros::Publisher pub("out", &out_msg);

std_msgs::Float32 err_msg;
ros::Publisher pub1("err", &err_msg);

std_msgs::Float32 gas_msg;
ros::Publisher pub2("gas", &gas_msg);


void setpointCallback(const std_msgs::Float32& msg){
  Setpoint = msg.data;
}
void encoder1Callback(const std_msgs::Float32& msg){
  Input2 = msg.data;// * 60.0 / (PI * wheel_diameter); // Convert m/s to RPM
}
void encoder2Callback(const std_msgs::Float32& msg){
  Input = msg.data; //* 60.0 / (PI * wheel_diameter); // Convert m/s to RPM
}

ros::Subscriber<std_msgs::Float32> sub_setpoint("setpoint_rpm", setpointCallback);
ros::Subscriber<std_msgs::Float32> sub_encoder1("encoder1_value", encoder1Callback);
ros::Subscriber<std_msgs::Float32> sub_encoder2("encoder2_value", encoder2Callback);

void setup() {
  // Initialize DAC
  dac.begin(0x60);

  // Inisialisasi serial
  Serial.begin(115200);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 4095); // Set PID output limits between 0 and 4095

  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub_setpoint);
  nh.subscribe(sub_encoder1);
  nh.subscribe(sub_encoder2);
  nh.advertise(pub);
  nh.advertise(pub1);
  nh.advertise(pub2);
}

void loop() {
  int adcdata = analogRead(A17);
  float voltage = adcdata * (3.3 / 1023.0);
  //Serial.println(voltage);
  // Compute PID
//  Setpoint = 10.0;
  myPID.Compute();
 /* if( (Setpoint - Input)<1600){
    Output += 1700;
  }*/
  //Output = 500+Output;
  mvo = map(Output, 0, 200, 1600, 4090);
  // Control Motor Speed using DAC
  if (mvo>4091) {
    mvo = 4090;
  }
  if (mvo<0){
    mvo = 0;
  }
  dac.setVoltage(0,false);
  //dac.setVoltage((uint16_t)mvo, false); // Set DAC voltage based on PID output directly
  gas_msg.data = voltage;
  out_msg.data = mvo;
  err_msg.data = Setpoint-Input;
  //pub.publish(&out_msg);
  //pub1.publish(&err_msg);
  pub2.publish(&gas_msg);
  nh.spinOnce(); 
  delay(1);
}

// Steering Control, Node 1 : ROS Serial
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <PID_v1.h>  

ros::NodeHandle nh;

#define RE_DE_PIN 2   // Control the RS485 mode (transmit/receive)
byte buffer[9];  // Buffer to store incoming bytes, adjust size based on expected message length
int bufferIndex = 0, encoderPosition = 0, angleValue = 0; 
int rpwm = 23, lpwm = 22, ren = 21, len = 20;
double currentAngle, controlSignal, targetAngle = 48; 
double kp = 0.07, ki = 0.009, kd = 0.90; // PID tuning parameters  
double previousError = 0, integral = 0; // For derivative and integral
int minPWM = 1800, maxPWM = 4095;
double previousTime = 0, IAE = 0.0, ISE = 0.0, ITAE = 0.0, ITSE = 0.0; 

// Publisher
std_msgs::Float32 msg;
std_msgs::Float32 msgc;
std_msgs::Float32 msge;
std_msgs::String msg1;
ros::Publisher perform("/topic1", &msg1);  // Publish to /topic1
ros::Publisher curra("/curra", &msgc);  // Publish to /topic1
ros::Publisher errora("/errora", &msge);  // Publish to /topic1
//ros::Publisher perform("/topic1", &msg1);  // Publish to /topic1

PID myPID(&currentAngle, &controlSignal, &targetAngle, kp, ki, kd, DIRECT); // Create PID Controller 

void messageCallback(const std_msgs::Float32& received_msg) {
  targetAngle = received_msg.data;  // Extract the message string

  // Convert the received message to a float
  //targetAngle = message.toFloat();
  //nh.loginfo(("Updated targetAngle: " + String(targetAngle)).c_str());
}

// Subscriber declaration for /topic2
ros::Subscriber<std_msgs::Float32> sub("/stir", &messageCallback);  // Subscribe to /topic2

void setup() {
  nh.initNode();          // Initialize the node
  nh.advertise(errora);   // Advertise the error angle
  nh.advertise(curra);   // Advertise the current angle
  nh.advertise(perform);  // Advertise the pid performance error
  nh.subscribe(sub);      // Subscribe to the topic
  Serial.begin(19200);           
  Serial1.begin(19200);
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(RE_DE_PIN, OUTPUT);
  analogWriteResolution(12);   // Set PWM resolution to 12 bits
  digitalWrite(RE_DE_PIN, LOW);  // Start in receive mode
  myPID.SetMode(AUTOMATIC);  
  previousTime = millis(); // Initialize the time 
}

void loop(){
  digitalWrite(ren, HIGH);
  digitalWrite(len, HIGH);
  digitalWrite(RE_DE_PIN, LOW);  // Enable receiver

  while (Serial1.available()){
    byte incomingByte = Serial1.read();   // Read data from RS485
    buffer[bufferIndex++] = incomingByte;  
    if (bufferIndex >= sizeof(buffer)){   // Check if we have enough data for processing
      processBuffer(buffer);
      bufferIndex = 0;                    // Reset index for next message
    }
  }

  double errorAngle = targetAngle - currentAngle; // Calculate the error between the target angle and the current angle
  integral += errorAngle * 0.05;
  double derivative = (errorAngle - previousError)/0.05;
  
  controlSignal = (kp*errorAngle) + (ki*integral) + (kd*derivative); // Proportional control: Speed based on the magnitude of error
  controlSignal = constrain(controlSignal, minPWM, maxPWM); // Constrain control signal between minPWM and maxPWM
  
  if (errorAngle < - 0.5) { // Rotate clockwise if the current angle is less than the target
    analogWrite(rpwm, controlSignal);
    analogWrite(lpwm, 0);

  } 
  else if (errorAngle > 0.5) { // Rotate counterclockwise if the current angle is greater than the target
    analogWrite(rpwm, 0);
    analogWrite(lpwm, controlSignal);

  } 
  else if (abs (errorAngle < 0.5)) { // Stop the motor if the target is reached (within a small tolerance)
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
  
  myPID.Compute(); // Compute the PID output
  previousError = errorAngle;
  unsigned long currentTime = millis();  
  double dt = (currentTime - previousTime) / 1000.0; // Time in seconds  

  IAE += fabs(errorAngle) * dt; // IAE  // Update performance indices
  ISE += pow(errorAngle, 2) * dt; // ISE  
  ITAE += fabs(errorAngle) * (dt * (currentTime - previousTime) / 1000.0); // ITAE (weighted by time)  
  ITSE += pow(errorAngle, 2) * (dt * (currentTime - previousTime) / 1000.0); // ITSE (weighted by time)  

  previousTime = currentTime; // Update previous time   
  
  // Create the message with all double variables
  String message = String("Target Angle: ") + String(targetAngle, 4) +  // Use 4 decimal places
                    String("  Current Angle: ") + String(currentAngle, 4) + 
                    String("  Error Angle: ") + String(errorAngle, 4) + 
                    String("  PWM: ") + String(controlSignal, 4) + 
                    String("  IAE: ") + String(IAE, 4) + 
                    String("  ISE: ") + String(ISE, 4) + 
                    String("  ITAE: ") + String(ITAE, 4) + 
                    String("  ITSE: ") + String(ITSE, 4);

  msgc.data = currentAngle;    // Set the message data
  msge.data = errorAngle;    // Set the message data
  msg1.data = message.c_str();    // Set the message data
  perform.publish(&msg1);   // Publish the message    
  curra.publish(&msgc);   // Publish the message    
  errora.publish(&msge);   // Publish the message    
  nh.spinOnce();    // Handle incoming messages            
  delay(50);  // Publish every second
  /*
  Serial.print("Target Angle: ");
  Serial.print(targetAngle);
  Serial.print("  ");
  Serial.print("Current Angle: ");
  Serial.print(currentAngle);
  Serial.print("  ");
  Serial.print("Error Angle: ");
  Serial.print(errorAngle);
  Serial.print("  ");
  Serial.print("PWM: ");
  Serial.print(controlSignal);
  Serial.print("IAE: ");  
  Serial.print(IAE);  
  Serial.print("  ");
  Serial.print("ISE: ");  
  Serial.print(ISE);  
  Serial.print("  ");
  Serial.print("ITAE: ");  
  Serial.print(ITAE);
  Serial.print("  ");  
  Serial.print("ITSE: ");  
  Serial.println(ITSE);  

  delay(50);
  */
}

void processBuffer(byte *buf){
  if (buf[0] == 171 && buf[1] == 205 && buf[2] == 5){ // Check for the expected start sequence
    encoderPosition = buf[3];                         //Extract position value (random)
    angleValue = buf[4];                              //Extract angle value (random)
    if (buf[5] == 0 && buf[6] == 255){                // Check the expected ending bytes
      byte sumCheck = buf[7];                         // Sum check value
      byte xorCheck = buf[8];                         // XOR check value
      byte endByte = buf[9];                          // Potential 3D byte (or any other)
      angleConversion();
    } 
  }
}

void angleConversion() {
  if (angleValue < 0) angleValue = 0; // Ensure the value is within the 0-255 range
  if (angleValue > 255) angleValue = 255;

  switch (encoderPosition) {
    case 0:
      currentAngle = (angleValue / 255.0f) * 22.5f;
      break;
    case 1:
      currentAngle = (angleValue / 255.0f) * (45.0f - 22.6f) + 22.6f;
      break;
    case 2:
      currentAngle = (angleValue / 255.0f) * (67.5f - 45.1f) + 45.1f;
      break;
    case 3:
      currentAngle = (angleValue / 255.0f) * (90.0f - 67.6f) + 67.6f;
      break;
    case 4:
      currentAngle = (angleValue / 255.0f) * (112.5f - 90.1f) + 90.1f;
      break;
    case 5:
      currentAngle = (angleValue / 255.0f) * (135.0f - 112.6f) + 112.6f;
      break;
    case 6:
      currentAngle = (angleValue / 255.0f) * (157.5f - 135.1f) + 135.1f;
      break;
    case 7:
      currentAngle = (angleValue / 255.0f) * (180.0f - 157.6f) + 157.6f;
      break;
    case 8:
      currentAngle = (angleValue / 255.0f) * (202.5f - 180.1f) + 180.1f;
      break;
    case 9:
      currentAngle = (angleValue / 255.0f) * (225.0f - 202.6f) + 202.6f;
      break;
    case 10:
      currentAngle = (angleValue / 255.0f) * (247.5f - 225.1f) + 225.1f;
      break;
    case 11:
      currentAngle = (angleValue / 255.0f) * (270.0f - 247.6f) + 247.6f;
      break;
    case 12:
      currentAngle = (angleValue / 255.0f) * (292.5f - 270.1f) + 270.1f;
      break;
    case 13:
      currentAngle = (angleValue / 255.0f) * (315.0f - 292.6f) + 292.6f;
      break;
    case 14:
      currentAngle = (angleValue / 255.0f) * (337.5f - 315.1f) + 315.1f;
      break;
    case 15:
      currentAngle = (angleValue / 255.0f) * (360.0f - 337.6f) + 337.6f;
      break;
    default: // Optional: handle unexpected encoderPosition values
      currentAngle = 0;
      break;
  }
}

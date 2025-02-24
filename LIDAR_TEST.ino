#include <RPLidar.h>

#define RPLIDAR_RX_PIN 
#define RPLIDAR_TX_PIN 
#define RPLIDAR_MOTOR_CTRL_PIN 

//instance
RPLidar lidar; 

void setup() {
  Serial.begin(115200); 
  
  if (!lidar.begin(RPLIDAR_RX_PIN, RPLIDAR_TX_PIN, RPLIDAR_MOTOR_CTRL_PIN)) { 
    Serial.println("Failed to start lidar!"); 
    while(1); 
  } 
}

void loop() {
  lidar.startScan(); 

  while (lidar.hasNextPacket()) {
    RPLidar::rplidar_response_measurement_node_t node;
    if (lidar.getNextMeasurement(&node)) {
      int distance = node.distance_q2; 
      int angle = node.angle_q6_check; 

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" mm, Angle: ");
      Serial.println(angle);
    }
  }
}
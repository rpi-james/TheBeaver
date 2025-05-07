/*******************************************************************************************************
  File: LiDAR_XYmapConvert.ino

  Description:
      ESP32-based LiDAR packet manager for RPLIDAR C1, now maintaining a full 360° distance map.
        - Initializes Serial1 at 460800 baud to communicate with LiDAR.
        - Sends start/stop commands and verifies the 7‑byte handshake response.
        - Reads 84‑byte scan packets of 40 measurements each.
        - Updates a degreeDistances[360] array so every degree’s latest distance is stored.
        - Clears stale data after 5 consecutive failed packets.
        - Converts angle-distance readings to XY coordinates

  Author: 2025 Senior Design II ECE team
  Updated by: Nicholas Matter
  Last Modified: April 2025

  Version:
    0.3.0 - Polar to XY

  Dependencies:
    - Arduino core (Serial, Serial1, delay, readBytes)  
    - math.h      For sin() and cos() lookup tables

  Hardware:
    - RPLIDAR C1 on Serial1  
        - RX (pin 18) on LiDAR TX  
        - TX (pin 19) on LiDAR RX  
    - USB Serial debug at 115200 baud

  Changes:
    v0.3.0 
      - Switched from per-packet output to full-circle array output
      - Rounded angles to integer degrees to maintain consistent indexing
      - Added coordinate transformation to enable spatial mapping
      - Improved packet sync verification
      - Smoothed delta angle calculation using lastAngle to improve scan accuracy

*******************************************************************************************************/  

#include <math.h>               // Needed for sin and cos

#define RPSERIAL       Serial1
#define RPLIDARBAUD    460800
#define MSGSERIAL      Serial
#define MSGBAUD        115200

// Command lengths per the protocol.
#define startCmd_length   9
#define endCmd_length     2
#define response_length   7
#define packet_length     84

// LiDAR commands and expected response.
char startCmd[startCmd_length]   = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
char endCmd[endCmd_length]       = { 0xA5, 0x25 };
char response[response_length]   = { 0xA5, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 };
char response_buffer[response_length];

float lastAngle = -1;  // Previous packet's start angle

// Measurements per packet.
float packetAngles[40]; // Computed angle for each measurement in a packet
float packetDists[40];  // Distance values from the sensor (raw)

// Buffer for one packet.
char rawdata[packet_length];

// Here we maintain an array containing one distance per degree (0-359).
// When a new measurement is received for a degree, it overwrites the previous value.
float degreeDistances[360]; 

int noDataCount = 0;

void setup() {
  MSGSERIAL.begin(MSGBAUD);
  RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, 18, 19);
  
  // Initialize the full-circle distance array.
  for (int i = 0; i < 360; i++) {
    degreeDistances[i] = 0;  // 0 indicates no reading yet (or you could initialize to a max value).
  }
  
  MSGSERIAL.println("Initializing RPLIDAR C1...");
  startScan();
}

void loop() {
  int result = Scanner();
  
  // If a valid packet was processed, update the degree array.
  if (result == 0) {
    noDataCount = 0;
    // Update global per-degree distance array with the new packet's 40 measurements.
    for (int i = 0; i < 40; i++) {
      // Get the current angle measurement (rounded to an integer between 0-359).
      int deg = (int)round(packetAngles[i]) % 360;
      if (deg < 0)
        deg += 360;
      
      float d = packetDists[i];
      // Optionally, ignore 0 values.
      if (d != 0) {
        degreeDistances[deg] = d;
      }
    }
  }
  else {
    noDataCount++;
    // Clear bad data if we miss a few packets.
    if (noDataCount > 5) {
      while (RPSERIAL.available()) RPSERIAL.read();
      noDataCount = 0;
    }
  }
  
  // Output one measurement per degree for a full circle, converting polar->XY.
  for (int deg = 0; deg < 360; deg++) {
    // Only output if a measurement is available.
    float r = degreeDistances[deg];
    if (r > 0) {
      // Convert degrees to radians for trig
      float rad = deg * (PI / 180.0);
      // Compute Cartesian coordinates (in same units as r, e.g. millimeters)
      float x = r * cos(rad);
      float y = r * sin(rad);

      // Print angle, distance, and XY
      MSGSERIAL.print("Angle: ");
      MSGSERIAL.print(deg);
      MSGSERIAL.print(" | Dist: ");
      MSGSERIAL.print(r, 2);
      MSGSERIAL.print(" mm | x: ");
      MSGSERIAL.print(x, 2);
      MSGSERIAL.print(" mm | y: ");
      MSGSERIAL.print(y, 2);
      MSGSERIAL.println(" mm");
    }
  }
  //MSGSERIAL.println("-------------------------------");
  
  delay(10);
}

bool arrayCmp(char* array1, char* array2, int len) {
  for (int i = 0; i < len; i++) {
    if (array1[i] != array2[i])
      return false;
  }
  return true;
}

void endScan() {
  RPSERIAL.write(endCmd, endCmd_length);
  RPSERIAL.flush();
}

void startScan() {
  endScan();
  RPSERIAL.flush();
  delay(1000);
  
  RPSERIAL.write(startCmd, startCmd_length);
  MSGSERIAL.println("Starting LiDAR...");
  
  if (RPSERIAL.readBytes(response_buffer, response_length) == response_length) {
    if (arrayCmp(response_buffer, response, response_length)) {
      MSGSERIAL.println("LiDAR Initialized Successfully.");
    }
    else {
      MSGSERIAL.println("ERROR: Unexpected response from LiDAR.");
    }
  }
  else {
    MSGSERIAL.println("ERROR: No response from LiDAR.");
  }
}

int Scanner() {
  if (RPSERIAL.available() < packet_length)
    return -1;
  
  RPSERIAL.readBytes(rawdata, packet_length);
  
  // Verify sync bytes based on your protocol.
  char sync1 = rawdata[0] >> 4;
  char sync2 = rawdata[1] >> 4;
  if (((sync1 << 4) + sync2) != 0xA5)
    return -1;
  
  // Compute the packet's start angle.
  float startAngle = (((rawdata[3] & 0x7F) << 8) + (unsigned char)rawdata[2]) / 64.0;
  
  // Determine the angular separation between measurements.
  float dAngle = 360.0 / 40.0;
  if (lastAngle >= 0) {
    float delta = fmod((360.0 + startAngle - lastAngle), 360.0);
    if (delta >= 0.5 && delta <= 40.0)
      dAngle = delta / 40.0;
  }
  
  // Process the 40 measurements in the packet.
  for (int i = 0; i < 40; i++) {
    packetAngles[i] = fmod((startAngle + dAngle * i), 360.0);
    uint16_t rawDistance = (((unsigned char)rawdata[4 + 2 * i + 1]) << 8) |
                           ((unsigned char)rawdata[4 + 2 * i]);
    packetDists[i] = rawDistance;
  }
  
  lastAngle = startAngle;
  return 0;
}

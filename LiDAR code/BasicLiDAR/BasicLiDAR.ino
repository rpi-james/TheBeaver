/*******************************************************************************************************
  File: BasicLiDAR.ino

  Description:
      ESP32-based LiDAR packet parser for RPLIDAR C1.
        - Initializes Serial1 at 460800 baud to communicate with LiDAR.
        - Sends start/stop commands and verifies the 7‑byte handshake response.
        - Reads one 84‑byte scan packet and decodes 40 angle/distance pairs.
        - Outputs the first (angle, distance) measurement over USB Serial for debugging.

  Author: 2025 Senior Design II ECE team
  Updated by: Nicholas Matter
  Last Modified: March 2025

  Version:
    0.1.0 - Initial LiDAR parsing release

  Dependencies:
    - Arduino core (Serial, Serial1, delay, readBytes)  
    - No external libraries

  Hardware:
    - RPLIDAR C1 on Serial1  
        - RX (pin 18) on LiDAR TX  
        - TX (pin 19) on LiDAR RX  
    - USB Serial debug at 115200 baud

  Notes:
    - First LiDAR iteration—no changelog entries yet.

*******************************************************************************************************/  

#define RPSERIAL Serial1  
#define RPLIDARBAUD 460800
#define MSGSERIAL Serial  // Debug output
#define MSGBAUD 115200

#define startCmd_length 9
#define endCmd_length 2
#define response_length 7
#define packet_length 84

char startCmd[startCmd_length] = {0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}; // start command - the missing link
char endCmd[endCmd_length] = {0xA5,0x25};                                        // stop command
char response[response_length] = {0xA5,0x5A,0x54,0x00,0x00,0x40,0x85};           // inital lidar response after success
char response_buffer[response_length];                                           // buffer to store response from lidar

float lastAngle;              // for detecting full rotation
float angleList[40];
float distList[40];
char rawdata[packet_length];  // raw data packet - for debugging
bool newTurn = false;         // flag for new revolution start

void setup() {
    MSGSERIAL.begin(MSGBAUD);                         // debug output initialization
    RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, 18, 19);  // lidar initialization

    MSGSERIAL.println("Initializing RPLIDAR C1...");  //debug output
    startScan();
}

void loop() {
    if (Scanner() == 0) {                                           // packet scan attempt
        MSGSERIAL.print("Angle: "); MSGSERIAL.print(angleList[0]);
        MSGSERIAL.print(" | Distance: "); MSGSERIAL.println(distList[0]); // debug output - angle and distance pair scanned
    }
}

// function to compare two character arrays - verrify lidar initial response matches (ensure lidar starts correctly)
bool arrayCmp(char* array1, char* array2, int len) {
    for (int i = 0; i < len; i++) {
        if (array1[i] != array2[i]) return false;
    }
    return true;
}

// stop the LiDAR scan
void endScan() {
    RPSERIAL.write(endCmd, endCmd_length);
}

// Start the LiDAR scan
void startScan() {
    endScan();        //stop current scan
    RPSERIAL.flush(); // clean buffer
    delay(1000);

    RPSERIAL.write(startCmd, startCmd_length);  // send start command
    MSGSERIAL.println("Starting LiDAR...");     // debug output

    RPSERIAL.readBytes(response_buffer, response_length); // verify inital response
    if (!arrayCmp(response_buffer, response, response_length)) {
        MSGSERIAL.println("ERROR: LiDAR Initialization Failed! Retrying...");
        delay(1000);
        startScan(); // retry after start failure
    } else {
        MSGSERIAL.println("LiDAR Initialized Successfully.");
    }
}

// Scan and parse a single LiDAR scan packet
int Scanner() {
    if (RPSERIAL.available() < packet_length) return -1;  // wait for entire packet

    RPSERIAL.readBytes(rawdata, packet_length);

    char sync1 = rawdata[0] >> 4;
    char sync2 = rawdata[1] >> 4;
    char ChkSum = (0b1111 & rawdata[0]) + ((0b1111 & rawdata[1]) << 4);
    char S = rawdata[3] >> 7;
    float startAngle = (((0b1111111 & rawdata[3]) << 8) + rawdata[2]) / 64.0;

    if (S == 1) MSGSERIAL.println("Reset database");

    if (((sync1 << 4) + sync2) != 0xA5) {
        MSGSERIAL.println("INVALID DATA! Restarting...");
        startScan();
        return -1;
    }

    float dAngle;
    if (lastAngle > startAngle) {
        dAngle = (360.0 + startAngle - lastAngle) / 40.0;
        newTurn = true;
    } else {
        dAngle = (startAngle - lastAngle) / 40.0;
    }

    for (int i = 0; i < 40; i++) {
        angleList[i] = dAngle * i + startAngle;                               // calculate angles
        distList[i] = ((rawdata[i * 2 + 1 + 4]) << 8) + (rawdata[i * 2 + 4]); // calculate distances
    }

    lastAngle = startAngle;
    return 0;
}



///*
// parsePacket: read through 84 B and gather 40 angle,dist pair
readBytes(rawdata, 84);                                       // grab full packet
float startA = (((rawdata[3]&0x7F)<<8 | rawdata[2]) / 64.0);  // start angle
for (int i=0; i<40; i++)
  packetDists[i] = unpackDist(rawdata, i),                    // distance
  packetAngles[i] = fmod(startA + i*(360.0/40.0), 360.0);     // angles

// find closest forward x
float minX = MAX_DIST;                                         // init no obstacle
unsigned long now = millis();
for (int d=270; d<360; d++) if (now−timestamp[d]≤DATA_TIMEOUT)
  minX = min(minX, degreeDistances[d]*cosTable[d]);
for (int d=0;   d<=90;  d++) if (now−timestamp[d]≤DATA_TIMEOUT)
  minX = min(minX, degreeDistances[d]*cosTable[d]);


// smooth slowdown: map dist→throttle scale
float norm = constrain((minX−STOP_DIST)/(SLOW_DIST−STOP_DIST), 0, 1);  
float s   = norm*norm*(3−2*norm);                            // smoothstep
smoothedScale = scale_alpha*smoothedScale + (1−scale_alpha)*s;
int adjThr     = throttle_ramp_out * smoothedScale;          // final throttle
*/

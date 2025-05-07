/*******************************************************************************************************
  File: TheBeaverFinal.ino AKA (Final_LiDAR_MotorControl.ino)
  Description: 
      ESP32-based skid‑steer motor control using microsecond‑pulse servo outputs. Integrated LiDAR
      obstacle detection with lighting and audio feedback
          - Reads Flysky IBus channels for throttle, steering, lights, and LiDAR enable.
          - Applies a center deadzone, and maps inputs to four ESP32Servo channels
              - Forward, reverse and turning maneuvars 
          - Smooths RC inputs with exponential low pass filters (α=0.9).             
          - Initializes RPLIDAR on Serial1 (RX GPIO18, TX GPIO19)
              - 460800 baud and parses 360° scan packets.
          - Precomputes sine/cosine lookup tables for fast polar→Cartesian conversion.
          - Discards stale readings older than 100 ms (DATA_TIMEOUT).
          - Dynamically scales throttle
          - Controls running light, taillight, and beeper output.
         **New in v1.1.0:**
          - Added configurable 250Hz MAIN_LOOP_FREQ based timing with watchdog safe yield() delays.
          - Implemented LiDAR startScan() retry logic (4 attempts).
          - Introduced smoothstep function for smoother slowdown curves.
          - Added separate low-pass smoothing for LiDAR distance averaging and scale factor.
          - BlinkRunningLights() for LiDAR failure and OFF indicator.
          - Updated SLOW_DIST=1000 mm and STOP_DIST=500 mm thresholds.
          - Conditional DEBUG macro to enable/disable.
          - Added headlight control and function on channel 5
          
  Author: 2025 Senior Design II ECE team
  Updated by: Nicholas Matter
  Last Modified: 2 May 2025

  Version:
      1.1.0 - Final prototype deployment version
    
  Dependencies:
    - IBusBM.h        FlySky iBus protocol
    - ESP32Servo.h    Servo control library for ESP32 (us precision) 
    - math.h          For sin(), cos() lookup tables 
    
  Hardware:  
    - Flysky IBus RX on GPIO16 (Serial2 RX), TX not used  
    - Servo/ESC inputs on GPIO2 (RF), GPIO4 (LF), GPIO23 (RR), GPIO25 (LR)  
    - RPLIDAR C1 on Serial1 (RX GPIO18, TX GPIO19)
    - Headlight: GPIO26 (digital)
    - Running light: GPIO32 (digital) 
    - Taillight: GPIO33 (digital)
    - Beeper: GPIO27 (digital)  

  Changes:
    v1.1.0 (Tweaks from prototype deployment version)
      - Added configurable 250Hz MAIN_LOOP_FREQ based timing with watchdog safe yield() delays.
      - Implemented LiDAR startScan() retry logic (4 attempts).
      - Introduced smoothstep function for smoother slowdown curves.
      - Added separate low-pass smoothing for LiDAR distance averaging and scale factor.
      - BlinkRunningLights() indicator for LiDAR failure and LiDAR OFF switch.
      - Conditional DEBUG macro to enable/disable.
      - Added headlight control and function on channel 5
/*******************************************************************************************************/


// Include libraries
#include <IBusBM.h>         // FlySky IBus protocol library
#include <ESP32Servo.h>     // Servo control on ESP32
#include <math.h>           // Math functions (sin, cos, fmod)

// Configurations Options
/***************************************************************************************************/

// Enable Debug mode
#define DEBUG

//used for testing frequency
unsigned long last_loop_time = 0;     // Timestamp of last loop iteration (µs)
unsigned long loop_time_elapsed = 0;  // Duration of last loop (µs)
// Debug counter
unsigned long loopCounter = 0;        // Counts loops for periodic debug output

// Loop timing variables
#define MAIN_LOOP_FREQ 250.0          // hertz = cycles/second (20% headroom)
long int loop_interval_us = (1.0/MAIN_LOOP_FREQ) * 1000000; // time to loop (us)

// LiDAR Definitions and Global Variable
/***************************************************************************************************/
#define LIDAR_RX_PIN 19         // connects to lidar RX (green)
#define LIDAR_TX_PIN 18         // connects to lidar TX (yellow)
#define RPSERIAL Serial1        // Hardware serial port for LiDAR
#define RPLIDARBAUD 460800      // LiDAR communication baud rate (per datasheet)
#define LIDAR_MSGSERIAL Serial  // USB serial for debug messages

// LiDAR packet lengths (bytes)
#define startCmd_length 9       // number of bytes in start command
#define endCmd_length 2         // number of bytes in end command 
#define response_length 7       // expected lidar response byte length
#define packet_length 84        // length of full packet to be parsed 
#define DATA_TIMEOUT 100        // Time until reading is ignored (ms)

// LiDAR commands
const char startCmd[startCmd_length]   = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
const char endCmd[endCmd_length]       = { 0xA5, 0x25 };
const char response[response_length]   = { 0xA5, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 };  // Expected response
char response_buffer[response_length]; // Buffer to hold LiDAR header response

// LiDAR data arrays and buffers
float lastAngle = -1;                  // Last starting angle read from packet
float packetAngles[40];                // Angles for each distance measurement in packet
float packetDists[40];                 // Raw distances from LiDAR packet
char rawdata[packet_length];           // Buffer to hold raw packet bytes
float degreeDistances[360];            // Latest distance for each degree
unsigned long degreeTimestamp[360];    // Timestamp to prevent stale readings
int noDataCount = 0;                   // Counter for empty/failed packets
bool lidarOFF = false;                 // Flag to control lights when lidar fails to start

// Motor, Lights, and IBus Definitions
/***************************************************************************************************/
#define IBUS_RX_PIN 16      // ESP32 pin receiving IBus data
#define IBUS_TX_PIN 17      // Unused TX for IBus (set to -1 in Serial2.begin)

// Motor output pins
#define RFMotor 2           // Servo pin: front right
#define LFMotor 4           // Servo pin: front left
#define RRMotor 23          // Servo pin: rear right
#define LRMotor 25          // Servo pin: rear left

// Lighting and beeper pins
#define Headlight 26        // Headlights pin
#define Runninglight 32     // Running lights pin
#define Taillight 33        // Taillights pin
#define Beeper 27           // Beeper pin

// Controller stick positions
#define THROTTLE_DEADZONE 10  // Minimum stick to register
#define TURN_DEADZONE 10      // Minimum steering to register
#define CENTER 1500           // Neutral PWM value
#define MIN 1000              // Minimum PWM from receiver
#define MAX 2000              // Maximum PWM from receiver

// Create IBus object and servo objects
IBusBM ibus;
Servo RFServo;
Servo LFServo;
Servo RRServo;
Servo LRServo;

// IBus channels
#define THR_CH 1
#define STR_CH 0
#define SER_CH 2
#define EXT_CH 3

// Receiver channel initialization
int ch1 = CENTER, ch2 = CENTER, ch3 = CENTER, ch4 = CENTER;
int ch5 = CENTER, ch6 = CENTER, ch7, ch8, ch9, ch10, ch11, ch12;

// Control variables
int throttle;         // Throttle output value (-100 to 100)
int steer;            // Steering output value (-100 to 100)
int headlights;       // Headlight PWM value

//Smoothing parameters
/***************************************************************************************************/
float throttle_smoothed    = 0.0;     // Filtered throttle input
int   steer_smoothed       = 0;       // Filtered steering input

//Throttle smoothing
const float throttle_alpha = 0.90;    // first‑order jitter filter for throttle 
const float steer_alpha    = 0.90;    // first‑order jitter filter for steering

// LiDAR slowdown and average smoothing
static float smoothedScale = 1.0;     // Smoothed scale factor for slowdown
const float scale_alpha   = 0.98;     // first‑order jitter filter for lidar scale
static float smoothedDist = 12000.0;  // var for dist after smoothing (start at max distance)
const float dist_alpha   = 0.50;      // first‑order jitter filter for lidar averaging (0 = no smoothing, 1 = infinite smoothing)

// Trig tables (lookups)
float sinTable[360];
float cosTable[360];


/***************************************************************************************************/

// Setup function that runs once as the ESP starts up
/***************************************************************************************************/
void setup() {
  Serial.begin(115200); // Serial debug

  // Precompute trig values for each degree in lookup table
  for (int d = 0; d < 360; d++) {
    float rad = d * (PI / 180.0);    // Convert degree to radians
    sinTable[d] = sin(rad);          // Compute sine vlaues
    cosTable[d] = cos(rad);          // Compute coside values
    degreeDistances[d] = 0;          // Init distances to 0
    degreeTimestamp[d] = 0;          // Iit timestamps to 0
  }

  // LiDAR init
  RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN); //Init LiDAR
  if (!startScan()) {               // Attempt LiDAR start scanning 
    Serial.println("LiDAR failed to initialize on power-up."); // Debug
    lidarOFF = true;                // LiDAR failed to start - set flag
  }

  // IBus & servos attach
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
  ibus.begin(Serial2, 1);
  RFServo.attach(RFMotor);
  LFServo.attach(LFMotor);
  RRServo.attach(RRMotor);
  LRServo.attach(LRMotor);

  // Lights & beeper outputs
  pinMode(Headlight, OUTPUT);
  pinMode(Runninglight, OUTPUT);
  pinMode(Taillight, OUTPUT);
  pinMode(Beeper, OUTPUT);
  digitalWrite(Taillight, HIGH); // Set taillights on

  // Initialize loop timing
  last_loop_time = micros();
} // end of setup()


// Loop that repeats forever within the ESP after setup runs once
/***************************************************************************************************/
void loop() {

  // Blink running lights when lidar off - DANGER MODE
  if (lidarOFF) {
    blinkRunningLights();
  }

  // Map channels
  read_receiver(&ch1, &ch2, &ch3, &ch4,
                &ch5, &ch6, &ch7, &ch8,
                &ch9, &ch10, &ch11, &ch12);

  // Map raw channel values with deadzone
  throttle = constrain(map(ch1, MIN, MAX, -110, 110), -100, 100);
  if (abs(throttle) < THROTTLE_DEADZONE) throttle = 0;    // Apply deadzone
  steer = constrain(map(ch2, MIN, MAX, -110, 110), -100, 100);

  // Toggle headlights with controller channel 5
  bool headlightsOn = (ch5 > CENTER);
  digitalWrite(Headlight, headlightsOn ? HIGH : LOW);   // Headlights on/off

  // Parse lidar packet (0 means success)
  int result = parsePacket();          // Read and parse packet
  if (result == 0) {                   // Packet parse success
    noDataCount = 0;                   // Reset counter
    // Distribute 40 distance-angle pairs
    for (int i = 0; i < 40; i++) {
      int deg = (int)round(packetAngles[i]) % 360; // Normalize angle to 0-359
      if (deg < 0) deg += 360;
      float d = packetDists[i];         // Get distance
      if (d != 0) {
        degreeDistances[deg] = d;       // Store distance for each angle  
        degreeTimestamp[deg] = millis();// Mark timestamp
      }
    }
  } 
  else {
    // Count failures; every 5 misses, clear buffer to resync
    noDataCount++;
    if (noDataCount > 5) {
      while (RPSERIAL.available())
        RPSERIAL.read(); // Clear buffer
      noDataCount = 0;   // Reset counter
    }
  }
  
  // Lidar slow/stop thresholds 
  const float SLOW_DIST = 1000                 // Slowdown start distance (mm)
  const float STOP_DIST = 500                  // Stop threshold distance (mm)
  const float ROBOT_HALF_WIDTH = 400.0;        // Half robot width for full coverage (mm)
  const float DIF_DIST = SLOW_DIST - STOP_DIST;// Effective slowdown range (mm)
  float effectiveFrontDist = 12000;            // Max range initial value (mm)

  // Toggle LiDAR with controller channel
  bool lidarEnabled = (ch6 > CENTER);
  if (lidarEnabled) {                          // If LiDAR-based avoidance enabled
    digitalWrite(Runninglight, HIGH);          // Solid running light to indicate active
    unsigned long now = millis();              // Time reference for staleness
    
    // Left area scan: 275 to compensate for drywall attachment
    for (int deg = 275; deg < 360; deg++) {
      if (now - degreeTimestamp[deg] >= DATA_TIMEOUT) continue; // Skip stale data
      float r = degreeDistances[deg];          // Distance reading
      if (r > 0) {                             // Valid reading
        float x = r * cosTable[deg];           // Convert polar angle to X coordinate
        float y = r * sinTable[deg];           // Convert polar angle to Y coordinate
        if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
          effectiveFrontDist = x;              // Update nearest front obstacle diistance
        }
      }

    // Right area scan: 85 to compensate for drywall attachment
    for (int deg = 0; deg <= 85; deg++) {
      if (now - degreeTimestamp[deg] >= DATA_TIMEOUT) continue; // Skip stale data
      float r = degreeDistances[deg];          // Distance reading 
      if (r > 0) {                             // Valid reading
        float x = r * cosTable[deg];           // Convert polar angle to X coordinate
        float y = r * sinTable[deg];           // Convert polar angle to Y coordinate
        if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
          effectiveFrontDist = x;              // Update nearest front obstacle diistance
      }
    }
  }
  // Blink running lights when lidar off - DANGER MODE
  else {
    blinkRunningLights();
  }

  // LiDAR low pass filter averaging
  smoothedDist = dist_alpha * effectiveFrontDist + (1.0 - dist_alpha) * smoothedDist;
  effectiveFrontDist = smoothedDist;

  // Dynamic slowdown calculation
  float scaleFactor = 1.0;                    // Init slowdown factor
  if (effectiveFrontDist < SLOW_DIST) {       // If object within slowdown range 
    float norm = (effectiveFrontDist - STOP_DIST) / (DIF_DIST); // Normalize 0-1
    norm = constrain(norm, 0.0, 1.0);         // Constrain normalized value
    scaleFactor = norm*norm*(3.0 - 2.0*norm); // Compute smoothstep f(n) = 3n² – 2n³
  }

  // Smooth the LiDAR scale for gradual stop
  smoothedScale = smoothedScale * scale_alpha + scaleFactor * (1.0 - scale_alpha); // Low pass filter

  // Smooth inputs
  throttle_smoothed = throttle_smoothed * throttle_alpha + throttle * (1 - throttle_alpha); // Low pass filter
  steer_smoothed    = steer_smoothed    * steer_alpha    + steer    * (1 - steer_alpha);    // Low pass filter

  // Slow/Stop only on forward throttle
  int adjustedThrottle = throttle_smoothed;
  if (adjustedThrottle > 0) {                                    // If moving forward
    adjustedThrottle = (int)(throttle_smoothed * smoothedScale); // Apply scaling factor
  }
  
  #ifdef DEBUG
  // Debug printing every 5 loops
  loopCounter++;
  if (loopCounter % 5 == 0) {
    Serial.print("Throttle (raw): "); Serial.print(throttle_smoothed);
    Serial.print(" | Adjusted Throttle: "); Serial.print(adjustedThrottle);
    Serial.print(" | Steering: "); Serial.print(steer_smoothed);
    Serial.print(" | Effective Front X: "); Serial.print(effectiveFrontDist);
    Serial.print(" mm | ScaleFactor: "); Serial.println(scaleFactor);
    //Serial.println(headlightsOn);
  }
  #endif

  // Motor control: if within deadzone then stop
  if (abs(adjustedThrottle) < THROTTLE_DEADZONE && abs(steer_smoothed) < TURN_DEADZONE) {
    left_motors(0); 
    right_motors(0);
  } 
  // otherwise update motors
  else {
    left_motors(adjustedThrottle + steer_smoothed);
    right_motors(adjustedThrottle - steer_smoothed);
  }

  // Beeper active on reverse throttle; off otherwise
  if (throttle_smoothed < -THROTTLE_DEADZONE) digitalWrite(Beeper, HIGH);
  else digitalWrite(Beeper,LOW);

  // Loop frequency timer
  while (micros() - last_loop_time < loop_interval_us) {
    yield();                      // feed watchdog while waiting    
  }
  last_loop_time += loop_interval_us;

}// end of loop()
/***************************************************************************************************/


// LiDAR Functions
/***************************************************************************************************/
// Compares byte arrays - used to check lidar response
bool arrayCmp(const char* a, const char* b, int len) { 
  for (int i = 0; i < len; i++) // Loop over all bytes
    if (a[i] != b[i])          
      return false;             // Byte mismatch
  return true;                  // All bytes match
}

// ends lidar scanner with end command
void endScan() { 
  RPSERIAL.write(endCmd, endCmd_length);// Write end command 
  RPSERIAL.flush();                     // Clear serial buffer
}


// start lidar scanner with start command
bool startScan() {
  for (int attempt = 0; attempt < 4; ++attempt) { // try startup 4 times
    while (RPSERIAL.available()) RPSERIAL.read(); // drain old data
    endScan();                                    // reset LiDAR
    RPSERIAL.flush();                             // Clear serial buffer
    delay(50);                                    // Time to reset - can likely be eliminated (or less)
    RPSERIAL.write(startCmd, startCmd_length);    // Write start command
    if (RPSERIAL.readBytes(response_buffer, response_length) == response_length
        && arrayCmp(response_buffer, response, response_length)) { // Read and verify start response
      return true;  // lidar startup success
    }
    delay(100);     // Wait before retry
  }
  return false;     // lidar failed to start
}



//Function to parse full raw data packets
int parsePacket() {
  if (RPSERIAL.available() < packet_length)  // ensure full packet arrived
    return -1;                               // packet not full
  
  RPSERIAL.readBytes(rawdata, packet_length);// Read packet
  
  //Check sync bits
  char sync1 = rawdata[0] >> 4;              // Sync 1 is upper 4 bits of byte 0
  char sync2 = rawdata[1] >> 4;              // Sync 2 is upper 4 bits of byte 1
  if (((sync1 << 4) + sync2) != 0xA5)        // Combine and compare to expected 0xA5
    return -1;                               // Failure - sync does not match
  
  // Compute starting angle and angle increment (bytes 2 and 3)
  float startAngle = (((rawdata[3] & 0x7F) << 8) + (unsigned char)rawdata[2]) / 64.0;
  float dAngle = 360.0 / 40.0;            // DEFAULT angle increment per sample
  if (lastAngle >= 0) {                   // If previous angle exists
    float delta = fmod((360.0 + startAngle - lastAngle), 360.0); // Calculate actual change between scans - 0-360
    if (delta >= 0.5 && delta <= 40.0)    // Validate delta range 
      dAngle = delta / 40.0;              // MEASURED angle increment per sample
  }
  
  // Extract distance measurement
  // rawdata[4 + 2*i]   = distance LSB
  // rawdata[4 + 2*i+1] = distance MSB
  // Read low and high bytes, construct 16-bit distance
  for (int i = 0; i < 40; i++) {
    packetAngles[i] = fmod((startAngle + dAngle * i), 360.0);    // Compute angle for each measurement - 0-360
    //Combine MSB and LSB
    uint16_t rawDistance = (((unsigned char)rawdata[4 + 2 * i + 1]) << 8) | // Upper 8 bits of distance - MSB
                           ((unsigned char)rawdata[4 + 2 * i]); // Lower 8 bits of distance of distance - LSB
    packetDists[i] = rawDistance;  // Store raw distance (mm)
  }
  lastAngle = startAngle;          // Update last angle for next computation
  return 0;                        // Parse success
}

// Motor Control Functions
/***************************************************************************************************/
// Function to run the left motor channels
void left_motors(int left_throttle) {
  left_throttle = constrain(left_throttle, -100, 100);  // Clamp to valid range -100 to 100
  if (left_throttle == 0) {                             // If no throttle
    LFServo.writeMicroseconds(CENTER);                  // Set front motor to CENTER - 0 throttle value
    LRServo.writeMicroseconds(CENTER);                  // Set rear motor to CENTER - 0 throttle value
  }
  else if (left_throttle > 0) {                         // If positive throttle (forward)
    int val = map(left_throttle, 0, 100, CENTER, 2700); // Scale 0 to 100 PWM range
    LFServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE)); // Set front motor to PWM value (adjust for deadzone)
    LRServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE)); // Set rear motor to PWM value (adjust for deadzone)
  }
  else {                                                // Else negative throttle (reverse)
    int val = map(left_throttle, -100, 0, 300, CENTER); // Scale -100 to 0 PWM range
    LFServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE)); // Set front motor to PWM value (adjust for deadzone)
    LRServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE)); // Set rear motor to PWM value (adjust for deadzone)
  }
}

// Function to run the right motor channels
void right_motors(int right_throttle) {
  right_throttle = constrain(right_throttle, -100, 100);  // Clamp to valid range -100 to 100
  if (right_throttle == 0) {                              // If no throttle
    RFServo.writeMicroseconds(CENTER);                    // Set front motor to CENTER - 0 throttle value
    RRServo.writeMicroseconds(CENTER);                    // Set rear motor to CENTER - 0 throttle value
  }
  else if (right_throttle > 0) {                          // If positive throttle (forward)
    int val = map(right_throttle, 0, 100, CENTER, 2700);  // Scale 0 to 100 PWM range
    RFServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE)); // Set front motor to PWM value (adjust for deadzone)
    RRServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE)); // Set rear motor to PWM value (adjust for deadzone)
  }
  else {                                                  // Else negative throttle (reverse)
    int val = map(right_throttle, -100, 0, 300, CENTER);  // Scale -100 to 0 PWM range
    RFServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE)); // Set front motor to PWM value (adjust for deadzone)
    RRServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE)); // Set rear motor to PWM value (adjust for deadzone)
  }
}

// Function to read values from the receivers and set them to channels
void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4,
                   int *ch5, int *ch6, int *ch7, int *ch8,
                   int *ch9, int *ch10, int *ch11, int *ch12) {
  *ch1 = ibus.readChannel(THR_CH);                      // Read throttle channel
  if (*ch1 > 2000 || *ch1 < 1000) *ch1 = CENTER;        // Clip invalid values
  *ch2 = ibus.readChannel(STR_CH);                      // Read steering channel
  if (*ch2 > 2000 || *ch2 < 1000) *ch2 = CENTER;        // Clip invalid values
  *ch3 = ibus.readChannel(SER_CH);                      // Read servo channel (unused)
  if (*ch3 > 2000 || *ch3 < 1000) *ch3 = CENTER;        // Clip invalid values
  *ch4 = ibus.readChannel(EXT_CH);                      // Read aux channel (unused)
  if (*ch4 > 2000 || *ch4 < 1000) *ch4 = CENTER;        // Clip invalid values

  // Read aux channels; extras zeroed
  *ch5 = ibus.readChannel(4);                           // Headlights toggle channel
  *ch6 = ibus.readChannel(5);                           // LiDAR toggle channel
  *ch7 = 0; *ch8 = 0; *ch9 = 0; *ch10 = 0; *ch11 = 0; *ch12 = 0;
}

void blinkRunningLights() {
  unsigned long cycle1 = 2000;                          // Total cycle time (ms)      
  unsigned long delta1 = millis() % cycle1;             // Position in cycle
  if (delta1 < 1500) digitalWrite(Runninglight, HIGH);  // Turn on lights if 1.5s
  else if (delta1 >= 1500) digitalWrite(Runninglight, LOW); // Turn off lights if 1.5s to 2s
}
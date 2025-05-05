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

#define startCmd_length 9       // number of bytes in start command
#define endCmd_length 2         // number of bytes in end command 
#define response_length 7       // expected lidar response byte length
#define packet_length 84        // length of full packet to be parsed 
#define DATA_TIMEOUT 100        // Time until reading is ignored (ms)

const char startCmd[startCmd_length]   = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
const char endCmd[endCmd_length]       = { 0xA5, 0x25 };
const char response[response_length]   = { 0xA5, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 };  // Expected response
char response_buffer[response_length]; // Buffer to hold LiDAR header response

float lastAngle = -1;                  // Last starting angle read from packet
float packetAngles[40];                // Angles for each distance measurement in packet
float packetDists[40];                 // Raw distances from LiDAR packet
char rawdata[packet_length];           // Buffer to hold raw packet bytes
float degreeDistances[360];            // Latest distance for each degree
unsigned long degreeTimestamp[360];    // Timestamp to prevent stale readings
int noDataCount = 0;                   // Counter for empty/failed packets
bool lidarOFF = false;                 // var to control lights when lidar OFF

// Motor, Lights, and IBus Definitions
/***************************************************************************************************/
#define IBUS_RX_PIN 16      // ESP32 pin receiving IBus data
#define IBUS_TX_PIN 17      // Unused TX for IBus (set to -1 in Serial2.begin)
#define RFMotor 2           // Servo pin: front right
#define LFMotor 4           // Servo pin: front left
#define RRMotor 23          // Servo pin: rear right
#define LRMotor 25          // Servo pin: rear left

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
// USed channel initialization
int ch1 = CENTER, ch2 = CENTER, ch3 = CENTER, ch4 = CENTER;
int ch5 = CENTER, ch6 = CENTER, ch7, ch8, ch9, ch10, ch11, ch12;

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
  Serial.begin(115200); 

  // Precompute trig values for each degree in lookup table
  for (int d = 0; d < 360; d++) {
    float rad = d * (PI / 180.0);
    sinTable[d] = sin(rad);
    cosTable[d] = cos(rad);
    degreeDistances[d] = 0;   // init measurements to 0
    degreeTimestamp[d] = 0;   // init timestamp to 0
  }

  // LiDAR init
  RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  if (!startScan()) {
    Serial.println("LiDAR failed to initialize on power-up.");
    lidarOFF = true;
  }

  // IBus & servos
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
  ibus.begin(Serial2, 1);
  RFServo.attach(RFMotor);
  LFServo.attach(LFMotor);
  RRServo.attach(RRMotor);
  LRServo.attach(LRMotor);

  // Lights & beeper
  pinMode(Headlight, OUTPUT);
  pinMode(Runninglight, OUTPUT);
  pinMode(Taillight, OUTPUT);
  pinMode(Beeper, OUTPUT);
  digitalWrite(Taillight, HIGH);

  // Initialize loop timing
  last_loop_time = micros();
} // end of setup()


// Loop that repeats forever within the ESP after setup runs once
/***************************************************************************************************/
void loop() {

  if (lidarOFF) {
    // Blink running lights when lidar off - DANGER MODE
    blinkRunningLights();
  }

  // Map channels
  read_receiver(&ch1, &ch2, &ch3, &ch4,
                &ch5, &ch6, &ch7, &ch8,
                &ch9, &ch10, &ch11, &ch12);

  throttle = constrain(map(ch1, MIN, MAX, -110, 110), -100, 100);
  if (abs(throttle) < THROTTLE_DEADZONE) throttle = 0;    //Apply deadzone

  steer = constrain(map(ch2, MIN, MAX, -110, 110), -100, 100);

  bool headlightsOn = (ch5 > CENTER);
  digitalWrite(Headlight, headlightsOn ? HIGH : LOW);   //Headlights on/off

  // Parse lidar packet (0 means success)
  int result = parsePacket();
  if (result == 0) {
    noDataCount = 0;
    // Distribute 40 distance-angle pairs
    for (int i = 0; i < 40; i++) {
      int deg = (int)round(packetAngles[i]) % 360;
      if (deg < 0) deg += 360;
      float d = packetDists[i];
      if (d != 0) {
        degreeDistances[deg] = d;
        degreeTimestamp[deg] = millis();  // Mark timestamp
      }
    }
  } 
  else {
    // Count failures; every 5 misses, clear buffer to resync
    noDataCount++;
    if (noDataCount > 5) {
      while (RPSERIAL.available())
        RPSERIAL.read(); // Clear buffer
      noDataCount = 0;
    }
  }

  // Lidar on/off
  bool lidarEnabled = (ch6 > CENTER);
  // Lidar slow/stop thresholds 
  const float SLOW_DIST = 1000, STOP_DIST = 500, ROBOT_HALF_WIDTH = 400.0;
  const float DIF_DIST = SLOW_DIST - STOP_DIST;
  float effectiveFrontDist = 12000;

  // Polar to XY (if lidar on per channel 6)
  if (lidarEnabled) {
    digitalWrite(Runninglight, HIGH); // Running lights solid
    unsigned long now = millis();
    // 275 to compensate for drywall attachment
    for (int deg = 275; deg < 360; deg++) {
      if (now - degreeTimestamp[deg] >= DATA_TIMEOUT) continue;
      float r = degreeDistances[deg];
      if (r > 0) {
        float x = r * cosTable[deg];
        float y = r * sinTable[deg];
        if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
          effectiveFrontDist = x;
        }
      }

    // 85 to compensate for drywall attachment
    for (int deg = 0; deg <= 85; deg++) {
      if (now - degreeTimestamp[deg] >= DATA_TIMEOUT) continue;
      float r = degreeDistances[deg];
      if (r > 0) {
        float x = r * cosTable[deg];
        float y = r * sinTable[deg];
        if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
          effectiveFrontDist = x;
      }
    }
  }
  else {
    // Blink running lights when lidar off - DANGER MODE
    blinkRunningLights();
  }

  // LiDAR low pass filter averaging
  smoothedDist = dist_alpha * effectiveFrontDist + (1.0 - dist_alpha) * smoothedDist;
  effectiveFrontDist = smoothedDist;

  // Dynamic slowdown calculation
  float scaleFactor = 1.0;
  if (effectiveFrontDist < SLOW_DIST) {
    float norm = (effectiveFrontDist - STOP_DIST) / (DIF_DIST);
    norm = constrain(norm, 0.0, 1.0);
    //scaleFactor = norm; // linear slowdown
    scaleFactor = norm*norm*(3.0 - 2.0*norm); //smoothstep f(n) = 3n² – 2n³
  }

  // Smooth the LiDAR scale for gradual stop
  smoothedScale = smoothedScale * scale_alpha + scaleFactor * (1.0 - scale_alpha);

  // Smooth inputs
  throttle_smoothed = throttle_smoothed * throttle_alpha + throttle * (1 - throttle_alpha);
  steer_smoothed    = steer_smoothed    * steer_alpha    + steer    * (1 - steer_alpha);

  // Slow/Stop only on forward throttle
  int adjustedThrottle = throttle_smoothed;
  if (adjustedThrottle > 0) {
    adjustedThrottle = (int)(throttle_smoothed * smoothedScale);
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

  // Motor control: if within deadzone then stop; otherwise update motors
  if (abs(adjustedThrottle) < THROTTLE_DEADZONE && abs(steer_smoothed) < TURN_DEADZONE) {
    left_motors(0); 
    right_motors(0);
  } 
  else {
    left_motors(adjustedThrottle + steer_smoothed);
    right_motors(adjustedThrottle - steer_smoothed);
  }

  // Beeper active on reverse throttle
  if (throttle_smoothed < -THROTTLE_DEADZONE) digitalWrite(Beeper, HIGH);
  else digitalWrite(Beeper,LOW);

  // Loop frequency timer
  while (micros() - last_loop_time < loop_interval_us) {
    yield();                          
  }
  last_loop_time += loop_interval_us;

}// end of loop()
/***************************************************************************************************/


// LiDAR Functions
/***************************************************************************************************/
// Compares byte arrays - used to check lidar response
bool arrayCmp(const char* a, const char* b, int len) {
  for (int i = 0; i < len; i++) 
    if (a[i] != b[i]) 
      return false;
  return true;
}

//ends lidar scanner with end command
void endScan() { 
  RPSERIAL.write(endCmd, endCmd_length); 
  RPSERIAL.flush(); 
}


//start lidar scanner with start command
bool startScan() {
  for (int attempt = 0; attempt < 4; ++attempt) { // try startup 4 times
    while (RPSERIAL.available()) RPSERIAL.read(); // drain old data
    endScan(); 
    RPSERIAL.flush();                             // Clear serial buffers
    delay(50);                                    // Time to reset - can likely be eliminated (or less)
    RPSERIAL.write(startCmd, startCmd_length);
    if (RPSERIAL.readBytes(response_buffer, response_length) == response_length
        && arrayCmp(response_buffer, response, response_length)) {
      return true;  // lidar startup success
    }
    delay(100);
  }
  return false; // lidar failed to start
}



//Function to parse full raw data packets
int parsePacket() {
  if (RPSERIAL.available() < packet_length) // ensure full packet arrived
    return -1;
  
  RPSERIAL.readBytes(rawdata, packet_length);
  
  //Check sync bits
  char sync1 = rawdata[0] >> 4;
  char sync2 = rawdata[1] >> 4;
  if (((sync1 << 4) + sync2) != 0xA5)
    return -1;
  
  // Compute starting angle and angle increment
  float startAngle = (((rawdata[3] & 0x7F) << 8) + (unsigned char)rawdata[2]) / 64.0;
  float dAngle = 360.0 / 40.0;
  if (lastAngle >= 0) {
    float delta = fmod((360.0 + startAngle - lastAngle), 360.0);
    if (delta >= 0.5 && delta <= 40.0)
      dAngle = delta / 40.0;
  }
  
  // Extract distance measurement
  for (int i = 0; i < 40; i++) {
    packetAngles[i] = fmod((startAngle + dAngle * i), 360.0);
    uint16_t rawDistance = (((unsigned char)rawdata[4 + 2 * i + 1]) << 8) |
                           ((unsigned char)rawdata[4 + 2 * i]);
    packetDists[i] = rawDistance;
  }
  lastAngle = startAngle;
  return 0;
}

// Motor Control Functions
/***************************************************************************************************/
// Function to run the left motor channels
void left_motors(int left_throttle) {
  left_throttle = constrain(left_throttle, -100, 100);
  if (left_throttle == 0) {
    LFServo.writeMicroseconds(CENTER);
    LRServo.writeMicroseconds(CENTER);
  }
  else if (left_throttle > 0) {
    int val = map(left_throttle, 0, 100, CENTER, 2700);
    LFServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE));
    LRServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE));
  }
  else {
    int val = map(left_throttle, -100, 0, 300, CENTER);
    LFServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE));
    LRServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE));
  }
}

// Function to run the right motor channels
void right_motors(int right_throttle) {
  right_throttle = constrain(right_throttle, -100, 100);
  if (right_throttle == 0) {
    RFServo.writeMicroseconds(CENTER);
    RRServo.writeMicroseconds(CENTER);
  }
  else if (right_throttle > 0) {
    int val = map(right_throttle, 0, 100, CENTER, 2700);
    RFServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE));
    RRServo.writeMicroseconds(val - (10 * THROTTLE_DEADZONE));
  }
  else {
    int val = map(right_throttle, -100, 0, 300, CENTER);
    RFServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE));
    RRServo.writeMicroseconds(val + (10 * THROTTLE_DEADZONE));
  }
}

// Function to reac values from the receivers and set them to channels
void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4,
                   int *ch5, int *ch6, int *ch7, int *ch8,
                   int *ch9, int *ch10, int *ch11, int *ch12) {
  *ch1 = ibus.readChannel(THR_CH);
  if (*ch1 > 2000 || *ch1 < 1000) *ch1 = CENTER; // Clip invalid values
  *ch2 = ibus.readChannel(STR_CH);
  if (*ch2 > 2000 || *ch2 < 1000) *ch2 = CENTER;
  *ch3 = ibus.readChannel(SER_CH);
  if (*ch3 > 2000 || *ch3 < 1000) *ch3 = CENTER;
  *ch4 = ibus.readChannel(EXT_CH);
  if (*ch4 > 2000 || *ch4 < 1000) *ch4 = CENTER;

  // Read aux channels; extras zeroed
  *ch5 = ibus.readChannel(4);
  *ch6 = ibus.readChannel(5);
  *ch7 = 0; *ch8 = 0; *ch9 = 0; *ch10 = 0; *ch11 = 0; *ch12 = 0;
}

void blinkRunningLights() {
  unsigned long cycle1 = 2000;
  unsigned long delta1 = millis() % cycle1;
  if (delta1 < 1500) digitalWrite(Runninglight, HIGH);
  else if (delta1 >= 1500) digitalWrite(Runninglight, LOW);
}

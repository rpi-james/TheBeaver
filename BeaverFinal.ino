// Include libraries
#include <IBusBM.h>
#include <ESP32Servo.h>
#include <math.h>

// Configurations Options
/***************************************************************************************************/

// Enable Debug mode
#define DEBUG

//used for testing frequency
unsigned long last_loop_time = 0;
unsigned long loop_time_elapsed = 0;
// Debug counter
unsigned long loopCounter = 0;

// Loop timing variables
#define MAIN_LOOP_FREQ 250.0  // hertz = cycles/second (20% headroom)
long int loop_interval_us = (1.0/MAIN_LOOP_FREQ) * 1000000; // time to loop (us)

// LiDAR Definitions and Global Variable
/***************************************************************************************************/
#define LIDAR_RX_PIN 18
#define LIDAR_TX_PIN 19
#define RPSERIAL Serial1
#define RPLIDARBAUD 460800
#define LIDAR_MSGSERIAL Serial

#define startCmd_length 9
#define endCmd_length 2
#define response_length 7
#define packet_length 84
#define DATA_TIMEOUT 100 // Time until reading is ignored (failsafe)

const char startCmd[startCmd_length]   = { 0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };
const char endCmd[endCmd_length]       = { 0xA5, 0x25 };
const char response[response_length]   = { 0xA5, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 };  // Expected response
char response_buffer[response_length];

float lastAngle = -1;
float packetAngles[40];
float packetDists[40];
char rawdata[packet_length];
float degreeDistances[360];
unsigned long degreeTimestamp[360]; // Used to prevent stale readings
int noDataCount = 0; // Used to count empty packets

// Motor, Lights, and IBus Definitions
/***************************************************************************************************/
#define IBUS_RX_PIN 16
#define IBUS_TX_PIN 17
#define RFMotor 2
#define LFMotor 4
#define RRMotor 23
#define LRMotor 25

#define Headlight 26
#define Runninglight 32
#define Taillight 33
#define Beeper 27

// Controller stick positions
#define THROTTLE_DEADZONE 10
#define TURN_DEADZONE 10
#define CENTER 1500
#define MIN 1000
#define MAX 2000

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
int ch1 = CENTER, ch2 = CENTER, ch3 = CENTER, ch4 = CENTER;
int ch5 = CENTER, ch6 = CENTER, ch7, ch8, ch9, ch10, ch11, ch12;

int throttle;
int steer;
int headlights;
int hazardState = 0;
unsigned long hazardTime = 0; 

//Smoothing parameters
/***************************************************************************************************/
float throttle_smoothed    = 0.0;   // α‑filtered stick input
int   steer_smoothed       = 0;     // steering filter
float throttle_ramp_out    = 0.0;   // time ramp output
unsigned long lastRampMicros = 0;   // change in ramo

//Throttle smoothing and ramping
const float throttle_alpha = 0.98;   // first‑order jitter filter
const float steer_alpha    = 0.98;   
const float rampDuration   = 1.2;    // seconds to traverse full range
const float divZero_guard   = 1e-6;  // guard against zero dt

// LiDAR slowdown and average smoothing
static float smoothedScale = 1.0;
const float scale_alpha   = 0.98;   
static float smoothedDist = 12000.0;   // start at max distance
const float dist_alpha   = 0.90;      // 0 = no smoothing, 1 = infinite smoothing

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
    degreeDistances[d] = 0;
    degreeTimestamp[d] = 0;
  }

  // LiDAR init
  RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  startScan();

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
  digitalWrite(Runninglight, HIGH);
  digitalWrite(Taillight, HIGH);

  // Initialize loop & ramp timing
  last_loop_time = micros();
  lastRampMicros = micros();
} // end of setup()


// Loop that repeats forever within the ESP after setup runs once
/***************************************************************************************************/
void loop() {
  // unsigned long loopStart = micros();
  
  read_receiver(&ch1, &ch2, &ch3, &ch4,
                &ch5, &ch6, &ch7, &ch8,
                &ch9, &ch10, &ch11, &ch12);

  throttle   = constrain(map(ch1, MIN, MAX, -110, 110), -100, 100);
  if (abs(throttle) < THROTTLE_DEADZONE) throttle = 0;
  steer      = constrain(map(ch2, MIN, MAX, -110, 110), -100, 100);
  //headlights = constrain(map(ch5, MIN, MAX, -10, 265), 0, 255);
  bool headlightsOn = (ch5 > CENTER);
  digitalWrite(Headlight, headlightsOn ? HIGH : LOW);

  int result = parsePacket();
  if (result == 0) {
    noDataCount = 0;
    for (int i = 0; i < 40; i++) {
      int deg = (int)round(packetAngles[i]) % 360;
      if (deg < 0) deg += 360;
      float d = packetDists[i];
      if (d != 0) {
        degreeDistances[deg]   = d;
        degreeTimestamp[deg] = millis();
      }
    }
  } 
  else {
    noDataCount++;
    if (noDataCount > 5) {
      while (RPSERIAL.available())
        RPSERIAL.read(); // Clear buffer
      noDataCount = 0;
    }
  }

  // Lidar on/off
  bool lidarEnabled = (ch6 > CENTER);
  const float SLOW_DIST = 1000, STOP_DIST = 500, ROBOT_HALF_WIDTH = 406.0;
  const float DIF_DIST = SLOW_DIST - STOP_DIST;
  float effectiveFrontDist = 12000;

  // Polar to XY
  if (lidarEnabled) {
    digitalWrite(Runninglight, HIGH);
    hazardState = 0;
    hazardTime = millis();
    unsigned long now = millis();
    for (int deg = 270; deg < 360; deg++) {
      if (now - degreeTimestamp[deg] <= DATA_TIMEOUT) {
        float r = degreeDistances[deg];
        if (r > 0) {
          float x = r * cosTable[deg], y = r * sinTable[deg];
          if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
            effectiveFrontDist = x;
        }
      }
    }

    for (int deg = 0; deg <= 90; deg++) {
      if (now - degreeTimestamp[deg] <= DATA_TIMEOUT) {
        float r = degreeDistances[deg];
        if (r > 0) {
          float x = r * cosTable[deg], y = r * sinTable[deg];
          if (x > 0 && fabs(y) < ROBOT_HALF_WIDTH && x < effectiveFrontDist)
            effectiveFrontDist = x;
        }
      }
    }
  }
  else {
    // Blink running lights when lidar off - DANGER MODE
    unsigned long cycle = 3000;
    unsigned long delta = millis() % cycle;

    if (delta < 2000) digitalWrite(Runninglight, HIGH);
    else if (delta >= 2000) digitalWrite(Runninglight, LOW);
  }

  // LiDAR low pass filter averaging
  smoothedDist = dist_alpha * smoothedDist + (1.0 - dist_alpha) * effectiveFrontDist;
  effectiveFrontDist = smoothedDist;

  // Dynamic slowdown calculation
  float scaleFactor = 1.0;
  if (effectiveFrontDist < SLOW_DIST) {
    float norm = (effectiveFrontDist - STOP_DIST) / (DIF_DIST);
    norm = constrain(norm, 0.0, 1.0);
    //scaleFactor = norm;
    scaleFactor = norm*norm*(3.0 - 2.0*norm); //smoothstep f(n) = 3n² – 2n³
  }

  // Smooth the LiDAR scale for gradual stop
  smoothedScale = smoothedScale * scale_alpha + scaleFactor * (1.0 - scale_alpha);

  // Smooth inputs
  throttle_smoothed = throttle_smoothed * throttle_alpha + throttle * (1 - throttle_alpha);
  steer_smoothed    = steer_smoothed    * steer_alpha    + steer    * (1 - steer_alpha);

  unsigned long nowRamp = micros();
  float dt = (nowRamp - lastRampMicros) * 1e-6;
  lastRampMicros = nowRamp;
  if (dt <= 0) dt = divZero_guard;

  float delta = throttle_smoothed - throttle_ramp_out;
  float step  = delta * (dt / rampDuration);
  throttle_ramp_out += step;

  // Adjust throttle if object detected
  int adjustedThrottle = (int)throttle_ramp_out;
  if (adjustedThrottle > 0)   // only limit forward movement based on lidar readings
    adjustedThrottle = (int)(throttle_ramp_out * smoothedScale);

  #ifdef DEBUG
  // Throttle debug printing every 5 loops
  loopCounter++;
  if (loopCounter % 5 == 0) {
    Serial.print("Throttle (raw): "); Serial.print(throttle_smoothed);
    Serial.print(" | Adjusted Throttle: "); Serial.print(adjustedThrottle);
    Serial.print(" | Steering: "); Serial.print(steer_smoothed);
    Serial.print(" | Effective Front X: "); Serial.print(effectiveFrontDist);
    Serial.print(" mm | ScaleFactor: "); Serial.println(scaleFactor);
    //Serial.println(headlightsOn);
    /*
    unsigned long loopEnd = micros();
    
    unsigned long loopTime = loopEnd - loopStart;
    Serial.print("LOOP END — Time (µs): ");
    Serial.println(loopTime);
    */
  }
  #endif

  // Motor control: if within deadzone then stop; otherwise update motors
  if (abs(adjustedThrottle) < THROTTLE_DEADZONE && abs(steer_smoothed) < TURN_DEADZONE) {
    left_motors(0); 
    right_motors(0);
  } else {
    left_motors(adjustedThrottle + steer_smoothed);
    right_motors(adjustedThrottle - steer_smoothed);
  }

  if (throttle_smoothed < -THROTTLE_DEADZONE) digitalWrite(Beeper, HIGH);
  else digitalWrite(Beeper,LOW);

  // Loop frequency timer
  while (micros() - last_loop_time < loop_interval_us) {
    yield();                          
  }
  //last_loop_time += loop_interval_us;
  last_loop_time = micros();

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
void startScan() {
  endScan(); 
  RPSERIAL.flush(); 
  delay(100);

  RPSERIAL.write(startCmd, startCmd_length);
  RPSERIAL.readBytes(response_buffer, response_length);

  /*#ifdef DEBUG
  if (RPSERIAL.readBytes(response_buffer, response_length) == response_length) {
    if (arrayCmp(response_buffer, response, response_length)) 
       Serial.println("LIDAR Initialized Successfully.");
    else  
      Serial.println("ERROR: Unexpected response from LIDAR.");
  }
  else {
    Serial.println("ERROR: No response from LIDAR.");
  }
  #endif*/
}

//Function to parse full raw data packets
//places distance-angle pair into corresponding arrays
int parsePacket() {
  if (RPSERIAL.available() < packet_length)
    return -1;
  
  RPSERIAL.readBytes(rawdata, packet_length);
  
  char sync1 = rawdata[0] >> 4;
  char sync2 = rawdata[1] >> 4;
  if (((sync1 << 4) + sync2) != 0xA5)
    return -1;
  
  float startAngle = (((rawdata[3] & 0x7F) << 8) + (unsigned char)rawdata[2]) / 64.0;
  float dAngle = 360.0 / 40.0;
  
  if (lastAngle >= 0) {
    float delta = fmod((360.0 + startAngle - lastAngle), 360.0);
    if (delta >= 0.5 && delta <= 40.0)
      dAngle = delta / 40.0;
  }
  
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
  if (*ch1 > 2000 || *ch1 < 1000)
    *ch1 = CENTER;
  *ch2 = ibus.readChannel(STR_CH);
  if (*ch2 > 2000 || *ch2 < 1000)
    *ch2 = CENTER;
  *ch3 = ibus.readChannel(SER_CH);
  if (*ch3 > 2000 || *ch3 < 1000)
    *ch3 = CENTER;
  *ch4 = ibus.readChannel(EXT_CH);
  if (*ch4 > 2000 || *ch4 < 1000)
    *ch4 = CENTER;
  *ch5 = ibus.readChannel(4);
  *ch6 = ibus.readChannel(5);
  *ch7 = 0; *ch8 = 0; *ch9 = 0; *ch10 = 0; *ch11 = 0; *ch12 = 0;
}

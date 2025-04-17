#include <IBusBM.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------- LiDAR Definitions and Global Variables ----------
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

int noDataCount = 0;

// ---------- Motor Lights and IBus Definitions ----------
#define IBUS_RX_PIN 16
#define IBUS_TX_PIN 17
#define RFMotor 2
#define LFMotor 4
#define RRMotor 23
#define LRMotor 25

#define Headlight 20
#define Runninglight 32
#define Taillight 33
#define Beeper 35

// Controller stick positions
#define THROTTLE_DEADZONE 10
#define TURN_DEADZONE 10
#define CENTER 1500
#define MIN 1000
#define MAX 2000

int throttle;
int steer;
int headlights;
int throttle_smoothed = 0;
int steer_smoothed = 0;
float throttle_alpha = 0.90;
float steer_alpha = 0.9;

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

// Trig tables (lookups)
float sinTable[360];
float cosTable[360];

// Debug counter
unsigned long loopCounter = 0;

// ---------- CODE START ----------
void setup() {
  Serial.begin(115200);
  // Precompute trig values for each degree
  for (int d = 0; d < 360; d++) {
    float rad = d * (PI / 180.0);
    sinTable[d] = sin(rad);
    cosTable[d] = cos(rad);
    degreeDistances[d] = 0;
    degreeTimestamp[d] = 0;
  }

  // LiDAR init
  RPSERIAL.begin(RPLIDARBAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("Initializing LIDAR...");
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
}

void loop() {
  read_receiver(&ch1, &ch2, &ch3, &ch4,
                &ch5, &ch6, &ch7, &ch8,
                &ch9, &ch10, &ch11, &ch12);

  throttle   = constrain(map(ch1, MIN, MAX, -110, 110), -100, 100);
  steer      = constrain(map(ch2, MIN, MAX, 110, -110), -100, 100);
  headlights = constrain(map(ch5, MIN, MAX, -10, 265), 0, 255);

  int result = Scanner();
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
  const float SLOW_DIST = 200, STOP_DIST = 50, ROBOT_HALF_WIDTH = 406.0;
  float effectiveFrontDist = 12000;

  // Polar to XY
  if (lidarEnabled) {
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

  // Dynamic slowdown calculation
  float scaleFactor = 1.0;
  if (effectiveFrontDist < SLOW_DIST) {
    scaleFactor = (effectiveFrontDist - STOP_DIST) / (SLOW_DIST - STOP_DIST);
    if (scaleFactor < 0) scaleFactor = 0;
  }

  // Smooth inputs
  throttle_smoothed = throttle_smoothed * throttle_alpha + throttle * (1 - throttle_alpha);
  steer_smoothed    = steer_smoothed    * steer_alpha    + steer    * (1 - steer_alpha);

  // Adjust throttle if object detected
  int adjustedThrottle = throttle_smoothed;
  if (adjustedThrottle > 0)
    adjustedThrottle = (int)(throttle_smoothed * scaleFactor);

  // Throttle debug printing every 5 loops
  loopCounter++;
  if (loopCounter % 5 == 0) {
    Serial.print("Throttle (raw): ");
    Serial.print(throttle_smoothed);
    Serial.print(" | Adjusted Throttle: ");
    Serial.print(adjustedThrottle);
    Serial.print(" | Steering: ");
    Serial.print(steer_smoothed);
    Serial.print(" | Effective Front X: ");
    Serial.print(effectiveFrontDist);
    Serial.print(" mm | ScaleFactor: ");
    Serial.println(scaleFactor);
  }

  // Motor control: if within deadzone then stop; otherwise update motors
  if (abs(adjustedThrottle) < THROTTLE_DEADZONE && abs(steer_smoothed) < TURN_DEADZONE) {
    left_motors(0); 
    right_motors(0);
  } else {
    left_motors(adjustedThrottle + steer_smoothed);
    right_motors(adjustedThrottle - steer_smoothed);
  }

  if (throttle_smoothed < -THROTTLE_DEADZONE) digitalWrite(Beeper, HIGH);

  analogWrite(Headlight, headlights);

  delay(5);
}

// ---------- LiDAR functions ----------
// Compares byte arrays - used to check lidar response
bool arrayCmp(const char* a, const char* b, int len) {
  for (int i = 0; i < len; i++) {
    if (a[i] != b[i]) 
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
  Serial.println("Starting LIDAR...");

  if (RPSERIAL.readBytes(response_buffer, response_length) == response_length) {
    if (arrayCmp(response_buffer, response, response_length)) {
      Serial.println("LIDAR Initialized Successfully.");
    }
    else {
      Serial.println("ERROR: Unexpected response from LIDAR.");
    }
  }
  else {
    Serial.println("ERROR: No response from LIDAR.");
  }
}

int Scanner() {
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

// ---------- Motor and IBus functions ----------

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

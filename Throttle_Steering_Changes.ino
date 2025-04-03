#include <IBusBM.h>
#include <ESP32Servo.h>

// Flysky IBus Receiver on RX2
#define IBUS_RX_PIN 16 
#define IBUS_TX_PIN 17

// Motor Pins on ESP32
#define RFMotor 2
#define LFMotor 4
#define RRMotor 23
#define LRMotor 25
#define HeadLight 20
#define RunningLight 32
#define Taillights 33
#define Beeper 35


// Defines for Motor Throttle and Steering Calculations
#define THROTTLE_DEADZONE 20 // Where the robot will not be moving from center
#define TURN_DEADZONE 20
#define CENTER 1500 //Center value of controller and Servo.write
int throttle;
int steer;
int headlights;
// Smoothed values to reduce current draw
int throttle_smoothed = 0;
int steer_smoothed = 0;
// Smoothing alpha values
float throttle_alpha = 0.90;
float steer_alpha = 0.9;

// Create Serial instances
IBusBM ibus; //IBus object
// defines for uniformity across protocols
#define THR_CH 1
#define STR_CH 0
#define SER_CH 2
#define EXT_CH 3
#define MIN 1000
#define MAX 2000
// Radio channel integers - raw values from the radio
int ch1 = CENTER;  // set to middle value so crawler doesnt move on startup
int ch2 = CENTER; 
int ch3 = CENTER;
int ch4 = CENTER;
int ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12;

//Create Servo Objects for each motor
Servo RFServo;
Servo LFServo;
Servo RRServo;
Servo LRServo;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

    ibus.begin(Serial2, 1);  // iBUS object connected to serial2 RX2 pin using timer 1
    
    RFServo.attach(RFMotor);
    LFServo.attach(LFMotor);
    RRServo.attach(RRMotor);
    LRServo.attach(LRMotor);

    pinMode(Headlight,OUTPUT);
    pinMode(Runninglight,OUTPUT);
    pinMode(Taillight,OUTPUT);
    pinMode(Beeper,OUTPUT);
}

void loop() {
  read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);

  //Map then constrian the throttle and steering values to a range that is easy to think about
  throttle = constrain(map(ch1, MIN, MAX, -110, 110), -100, 100);
  steer = constrain(map(ch2, MIN, MAX, 110, -110), -100, 100);
  headlights = constrain(map(ch5,MIN,MAX,-10,110),0,100);
  // Avg the past throttle values to smooth the acceleration and possible spikes
  throttle_smoothed = (throttle_smoothed * (throttle_alpha)) + (throttle * (1-throttle_alpha));
  steer_smoothed = (steer_smoothed * (steer_alpha)) + (steer * (1-steer_alpha));

  // Calculate motor values
  if ((throttle_smoothed > -THROTTLE_DEADZONE && throttle_smoothed < THROTTLE_DEADZONE) && 
      (steer_smoothed > -TURN_DEADZONE && steer_smoothed < TURN_DEADZONE)){
    // deadzone to stop unwanted drift
    left_motors(0);
    right_motors(0);
  }
  else{ 
    // throttle mixing for tank/skid steering
    left_motors(throttle_smoothed + steer_smoothed);
    right_motors(throttle_smoothed - steer_smoothed);
  }
  
  // Debug Output
  Serial.print("Throttle: ");             Serial.print(throttle_smoothed);
  Serial.print(" | Steering: ");          Serial.println(steer_smoothed);
  delay(50);
}

void left_motors(int left_throttle){
  left_throttle = constrain(left_throttle,-100,100);

  if(left_throttle==0){
    LFServo.writeMicroseconds(CENTER);
    LRServo.writeMicroseconds(CENTER);
  }
  else if (left_throttle > 0){
    int val = map(left_throttle,0,100,CENTER,2700);
    LFServo.writeMicroseconds(val-(10*THROTTLE_DEADZONE));
    LRServo.writeMicroseconds(val-(10*THROTTLE_DEADZONE));
    Serial.print("Left Val: "); Serial.println(val);
  }
  else if (left_throttle < 0){
    int val = map(left_throttle,-100,0,300,CENTER);
    LFServo.writeMicroseconds(val+(10*THROTTLE_DEADZONE));
    LRServo.writeMicroseconds(val+(10*THROTTLE_DEADZONE));
  }
}

void right_motors(int right_throttle){
  right_throttle = constrain(right_throttle,-100,100);

  if(right_throttle==0){
    RFServo.writeMicroseconds(CENTER);
    RRServo.writeMicroseconds(CENTER);
  }
  else if (right_throttle > 0){
    int val = map(right_throttle,0,100,CENTER,2700);
    RFServo.writeMicroseconds(val-(10*THROTTLE_DEADZONE));
    RRServo.writeMicroseconds(val-(10*THROTTLE_DEADZONE));
    Serial.print("Right Val: "); Serial.println(val);
  }
  else if (right_throttle < 0){
    int val = map(right_throttle,-100,0,300,CENTER);
    RFServo.writeMicroseconds(val+(10*THROTTLE_DEADZONE));
    RRServo.writeMicroseconds(val+(10*THROTTLE_DEADZONE));
  }
}

void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4, int *ch5, int *ch6, int *ch7, int *ch8, int *ch9, int *ch10, int *ch11, int *ch12){
  // This section is called if the IBus protocol is selected
  *ch1 = ibus.readChannel(THR_CH);   // get latest value from channel 1
  if (*ch1 > 2000 || *ch1 < 1000){    // lost connection, set to defaults
    *ch1 = CENTER;
  }
  *ch2 = ibus.readChannel(STR_CH);   // get latest value from channel 2
  if (*ch2 > 2000 || *ch2 < 1000){    // lost connection, set to defaults
    *ch2 = CENTER;
  }
  *ch3 = ibus.readChannel(SER_CH);   // get latest value from channel 3
  if (*ch3 > 2000 || *ch3 < 1000){    // lost connection, set to defaults
    *ch3 = CENTER;
  }
  *ch4 = ibus.readChannel(EXT_CH);   // get latest value from channel 4
  if (*ch4 > 2000 || *ch4 < 1000){    // lost connection, set to defaults
    *ch4 = CENTER;
  }
  *ch5 = ibus.readChannel(4);  // get latest value from channel 5
  *ch6 = ibus.readChannel(5);  // get latest value from channel 6
  *ch7 = 0;
  *ch8 = 0;
  *ch9 = 0;
  *ch10 = 0;
  *ch11 = 0;
  *ch12 = 0;
}

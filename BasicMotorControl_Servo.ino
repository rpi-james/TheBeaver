/*******************************************************************************************************
  File: MotorDriving_RCPulse.ino
  Description: 
      ESP32-based skid‑steer motor control using  microsecond‑pulse servo outputs. 
          - Reads throttle (channel 2) and steering (channel 4) via Flysky IBus receiver
          - Applies a center deadzone, and maps inputs to four ESP32Servo channels
              - Forward, reverse and turning maneuvars 
          
  Author: 2025 Senior Design II ECE team
  Updated by: Tyler James
  Last Modified: 13 Febrary 2025

  Version:
      1.0.1
    
  Dependencies:
    - IBusBM.h        FlySky iBus protocol
    - ESP32Servo.h    Servo control library for ESP32 (us precision) 
    
  Hardware:  
    – Flysky IBus RX on GPIO16 (Serial2 RX), TX not used  
    – Servo/ESC inputs on GPIO2 (RF), GPIO4 (LF), GPIO23 (RR), GPIO25 (LR)  
    
  Changelog:
    v1.0.1 (2025‑02‑13)
      - Replaced RoboClaw commands with direct PWM via ESP32Servo  
      - Added DEADZONE handling around center for throttle & steering  
      - Refined differential steering logic for smoother turns  
/*******************************************************************************************************/

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


// Defines for Motor Throttle and Steering Calculations
#define DEADZONE 200 // Where the robot will not be moving or steering from center
#define CENTER 1500 //Center value of controller and Servo.write


// Create Serial instances
IBusBM ibus; //IBus object
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

}

void loop() {
    //if (ibus.read()) {
        int throttle = ibus.readChannel(1);  // Channel 2 (Throttle)
        int steer = ibus.readChannel(3)-CENTER;  // Channel 4 (Steering)

        if (throttle<1000 || throttle>2000) {throttle=CENTER;}
        if (steer<-500 || steer>500) {steer=0;}

        // Convert iBus range (1000-2000) to RoboClaw motor range (500 to 2500)
        int motorSpeed = map(throttle, 1000, 2000, 500, 2500);
        int turn = map(steer, -500, 500, -1000, 1000);

        // Calculate motor values
        int leftMotor=CENTER,rightMotor=CENTER;
        //If the thro
        if(motorSpeed<=(CENTER+DEADZONE) && motorSpeed>=(CENTER-DEADZONE)){
          leftMotor  = CENTER;
          rightMotor = CENTER;
        }
        else if(motorSpeed>(CENTER) && turn<(-DEADZONE)){ //Forward Left Turn
          leftMotor  = CENTER + turn;
          rightMotor = CENTER - turn;
        }
        else if(motorSpeed>(CENTER) && turn>(DEADZONE)){ //Forward Right Turn
          leftMotor  = CENTER + turn;
          rightMotor = CENTER - turn;
        }
        else if(motorSpeed<(CENTER) && turn<(-DEADZONE)){ //Reverse Left Turn
          leftMotor  = CENTER - turn;
          rightMotor = CENTER + turn;
        }
        else if(motorSpeed<(CENTER) && turn>(DEADZONE)){ //Forward Right Turn
          leftMotor  = CENTER - turn;
          rightMotor = CENTER + turn;
        }
        else if(motorSpeed>CENTER || motorSpeed<CENTER && turn>(-DEADZONE) && turn<DEADZONE){ // Forward  and Reverse Straight
          leftMotor  = motorSpeed;
          rightMotor = motorSpeed;
        }

        //Drive the motors using the servo objects
        RFServo.writeMicroseconds(rightMotor);
        LFServo.writeMicroseconds(leftMotor);
        RRServo.writeMicroseconds(rightMotor);
        LRServo.writeMicroseconds(leftMotor);

        // Debug Output
        Serial.print("Throttle: "); Serial.print(throttle);
        Serial.print(" | Steering: "); Serial.print(steer);
        Serial.print(" | Motor Speed: "); Serial.print(motorSpeed);
        Serial.print(" | Turn: "); Serial.print(turn);
        Serial.print(" | Left Motor: "); Serial.print(leftMotor);
        Serial.print(" | Right Motor: "); Serial.println(rightMotor);
        delay(50);
    //}
}

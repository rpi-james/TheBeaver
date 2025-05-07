/*******************************************************************************************************
  File: BasicMotorControl_RoboClaw.ino
  Description: 
      ESP32-based skid‑steer motor control for a Flysky RC transmitter using RoboClaw motor controllers. 
          - Reads throttle (channel 2) and steering (channel 4) via Flysky IBus receiver
          - Maps the 1000–2000 us range to –127 to +127 speed values
          - Issues Forward/Backward commands to M1 and M2 on the RoboClaw.

  Author: 2025 Senior Design II ECE team
  Updated by: Nicholas Matter
  Last Modified: 10 Febrary 2025

  Version:
      0.0.0 
    
  Dependencies:
    - IBusBM.h        FlySky iBus protocol
    - RoboClaw.h      RoboClaw Serial motor controller library  
    
  Hardware:  
    – Flysky IBus RX on GPIO16 (Serial2 RX), TX not used  
    – RoboClaw on Serial1 (TX GPIO4, RX GPIO2), address 128
    
  Notes:
    - No change log entries yet. This is the first iteration.  
    - For future updates and detailed history, see changelog
/*******************************************************************************************************/

#include <IBusBM.h>
#include "RoboClaw.h"

// Flysky IBus Receiver on RX2
#define IBUS_RX_PIN 16 

// RoboClaw Serial
#define ROBOCLAW_RX 2
#define ROBOCLAW_TX 4 
#define ROBOCLAW_ADDRESS 128  // Default RoboClaw address

// Create Serial instances
HardwareSerial roboclawSerial(1);
RoboClaw roboclaw(&roboclawSerial, 10000);
IBusBM ibus; //IBus object

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

    ibus.begin(Serial2,1);  // iBUS object connected to serial2 RX2 pin using timer 1

    roboclawSerial.begin(38400, SERIAL_8N1, ROBOCLAW_TX, ROBOCLAW_RX);
    roboclaw.begin(38400);
}

void loop() {
    //if (ibus.read()) {
        int throttle = ibus.readChannel(1);  // Channel 2 (Throttle)
        int steering = ibus.readChannel(3);  // Channel 4 (Steering)

        // Convert iBus range (1000-2000) to RoboClaw motor range (-127 to +127)
        int motorSpeed = map(throttle, 1000, 2000, -127, 127);
        int turn = map(steering, 1000, 2000, -127, 127);

        // Calculate motor values
        int leftMotor = motorSpeed + turn;
        int rightMotor = motorSpeed - turn;

        // Send commands to RoboClaw
        if (leftMotor >= 0)
            roboclaw.ForwardM1(ROBOCLAW_ADDRESS, leftMotor);
        else
            roboclaw.BackwardM1(ROBOCLAW_ADDRESS, -leftMotor);

        if (rightMotor >= 0)
            roboclaw.ForwardM2(ROBOCLAW_ADDRESS, rightMotor);
        else
            roboclaw.BackwardM2(ROBOCLAW_ADDRESS, -rightMotor);

        // Debug Output
        Serial.print("Throttle: "); Serial.print(throttle);
        Serial.print(" | Steering: "); Serial.print(steering);
        Serial.print(" | Left Motor: "); Serial.print(leftMotor);
        Serial.print(" | Right Motor: "); Serial.println(rightMotor);
    //}
}
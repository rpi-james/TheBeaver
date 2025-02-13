#include <IBusBM.h>
//#include "RoboClaw.h"
#include <ESP32Servo.h>

// Flysky IBus Receiver on RX2
#define IBUS_RX_PIN 16 
#define IBUS_TX_PIN 17

// Motor Pins on ESP32
#define MotorA 2
#define MotorB 4

// Defines for Motor Throttle and Steering Calculations
#define DEADZONE 200 // Where the robot will not be moving or steering from center
#define CENTER 1500 //Center value of controller and Servo.write
#define RANGE 400 // Range of values possible after deadzone


// Create Serial instances
IBusBM ibus; //IBus object
Servo ServoA;
Servo ServoB;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

    ibus.begin(Serial2, 1);  // iBUS object connected to serial2 RX2 pin using timer 1
    
    ServoA.attach(MotorA);
    ServoB.attach(MotorB);
}

void loop() {
    //if (ibus.read()) {
        int throttle = ibus.readChannel(1);  // Channel 2 (Throttle)
        int steer = ibus.readChannel(3)-CENTER;  // Channel 4 (Steering)

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
        ServoA.writeMicroseconds(leftMotor);
        ServoB.writeMicroseconds(rightMotor);
/*  
        // Send commands to RoboClaw
        if (leftMotor >= 0)
            roboclaw.ForwardM1(ROBOCLAW_ADDRESS, leftMotor);
        else
            roboclaw.BackwardM1(ROBOCLAW_ADDRESS, -leftMotor);

        if (rightMotor >= 0)
            roboclaw.ForwardM2(ROBOCLAW_ADDRESS, rightMotor);
        else
            roboclaw.BackwardM2(ROBOCLAW_ADDRESS, -rightMotor);
*/
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

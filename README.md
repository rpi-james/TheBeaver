## TheBeaver
Senior Design Project — ESP32-Based 4WD Robot with Obstacle Avoidance

# Overview
The Beaver is a four-motor skid-steer robot controlled via an RC transmitter (FlySky iBus) and powered by an ESP32. It integrates real-time LiDAR scanning using an RPLIDAR C1 for obstacle detection and autonomous slowdown. The project combines motor control, sensor fusion, and basic failsafe logic. It is complete with visual and audible indicators, utilizing always on running lights and taillights, controllable headlights, and a backup beeper.

# Features
- Throttle and steering input via FlySky iBus (Channels 2 and 4 - single stick control)
- LiDAR based obstacle detection with adjustable slow/stop distance
- Dynamic throttle scaling based on obstacle proximity
- RC channel 5 toggles LiDAR system and headlights - running lights blink if LiDAR turned off
- Custom smoothing filters for steering and throttle
- ESC control via PWM output to servo-compatible motor controllers

# Folder Structure
- `Motor Code/` — RC input reading and PWM-based motor control iterations
- `Lidar Code/` — ESP32-based LiDAR packet reader and distance mapping iterations
- `Integrated Robot/` — Combined system with obstacle avoidance, lights, and control logic iterations
- `TheBeaverFinal/` — Final combined system with obstacle avoidance, lights, and control logic for deployable prototype

# Hardware
- ESP32 (any dev board)
- FlySky iA6B/iA6 receiver
- RPLIDAR C1 (Serial1 at 460800 baud)
- 2x RoboClaw Motor Controllers (servo ESC or equivalent)
- Optional: LEDs, beeper, headlight control

# Setup
No external libraries needed. Flash the code directly to ESP32 using Arduino IDE or PlatformIO. Use standard wiring conventions and ensure 5V LiDAR is correctly powered.

# Usage
- Power on robot
- Use transmitter to drive (throttle = Ch2, steering = Ch4)
- LiDAR scans autonomously; robot slows/stops based on obstacles
- Channel 5 toggles LiDAR
- Channel 6 toggles headlights

# Notes
- See each folder’s `CHANGELOG.md` for version history
- Code is structured for readability and modularity
- Avoid running LiDAR without open space to test detection


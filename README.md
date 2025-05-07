## TheBeaver
Senior Design Project - ESP32-Based 4WD Robot with Obstacle Avoidance

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
- `Motor Code/` - RC input reading and PWM-based motor control iterations
- `Lidar Code/` - ESP32-based LiDAR packet reader and distance mapping iterations
- `Integrated Robot Code/` - Combined system with obstacle avoidance, lights, and control logic iterations
- `TheBeaverFinal/` - Final combined system with obstacle avoidance, lights, and control logic for deployable prototype

# Hardware
- ESP32 (any dev board)
- FlySky iA6B/iA6 receiver
- RPLIDAR C1 (Serial1 at 460800 baud)
- 2x RoboClaw Motor Controllers (servo ESC or equivalent)
- Optional: LEDs, beeper, headlight control

# Setup
No external libraries needed. Flash the code directly to ESP32 using Arduino IDE or PlatformIO. Use standard wiring conventions and ensure 5V LiDAR, ESP32, and FlySky receiver is correctly powered.

## Pinout (ESP32 Dev Board)
| Component                         | ESP32 Pin |
|-----------------------------------|-----------|
| Front Left motor ESC PWM          | GPIO 4    |
| Rear Left motor ESC PWM           | GPIO 25   |
| Front Right motor ESC PWM         | GPIO 2    |
| Rear Right motor ESC PWM          | GPIO 23   |
| FlySky iBus RX (UART2 RX)         | GPIO 16   |
| RPLIDAR C1 - ESP32 RX1 (Serial1)  | GPIO 18   |
| RPLIDAR C1 - ESP32 TX1 (Serial1)  | GPIO 19   |
| Headlight control (Channel 5)     | GPIO 26   |
| Running Lights                    | GPIO 32   |
| Taillights                        | GPIO 33   |
| Beeper                            | GPIO 27   |

Ensure all grounds are common and the 5 V supply is stable.

# Usage
- Power on robot
- Use transmitter to drive (throttle = Ch2, steering = Ch4) - right stick only
- LiDAR scans autonomously; robot slows/stops based on obstacles
- Channel 5 toggles headlights - left dial
- Channel 6 toggles LiDAR - right dial (running lights will blink when off or if LiDAR failed to startup)

# Notes
- See each folder’s `CHANGELOG.md` for version history
- Code is structured for readability and modularity
- Avoid running LiDAR without open space initially to test detection


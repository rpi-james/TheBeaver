# Changelog

Combined motor control and obstacle‐detection prototype for four‑motor skid‑steer robot.

## [1.1.0] – May 2025
### Added
- Configurable `MAIN_LOOP_FREQ` (250 Hz) timing with watchdog‐safe `yield()` delays.
- LiDAR `startScan()` retry logic (4 attempts).
- `smoothstep` function for smoother slowdown curves.
- Separate low‑pass smoothing for LiDAR distance averaging and scale factor.
- `BlinkRunningLights()` indicator for LiDAR failure and “LiDAR OFF” switch.
- Conditional `DEBUG` macro to enable/disable verbose output.
- Headlight control on RC channel 5.

## [1.0.0] – April 2025
### Added
- First motor + obstacle‑detection prototype.
  - Smoothing filters (α = 0.9) on throttle & steering inputs.
  - Lighting & beeper control.
  - LiDAR obstacle avoidance:
    - Enable/disable via RC channel.
    - Precomputed sine/cosine lookup tables for fast polar to Cartesian transforms.
    - `DATA_TIMEOUT` and `noDataCount` logic: auto‑flush on errors.
    - Dynamic throttle scaling based on `SLOW_DIST` (200 mm) & `STOP_DIST` (50 mm).
    - Configurable `ROBOT_HALF_WIDTH` for detection sizing.
  - Refactored into helper functions:
    - `startScan()`, `endScan()`, `Scanner()`, `left_motors()`, `right_motors()`, `read_receiver()`.
  - Enhanced debug every five loops: raw vs. adjusted throttle/steering, `effectiveFrontDist`, `scaleFactor`.

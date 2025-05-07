# Changelog

Tracks changes to the ESP32-based skid‑steer motor control using a FlySky RC transmitter and RoboClaw (later PWM) controllers.

## [0.0.2] – 2025‑03‑20
### Added
- Throttle and steering exponential smoothing.
- Integrated channel 5 control for headlight activation (future expansion).
- Digital outputs for auxiliary functions.
- Refined motor‑mapping logic with deadzone handling per side.
- Reorganized channel definitions for protocol consistency.

## [0.0.1] – 2025‑02‑13
### Changed
- Replaced RoboClaw commands with direct PWM via `ESP32Servo`.
- Added DEADZONE handling around center for throttle & steering.
- Refined differential steering logic for smoother turns.

## [0.0.0] – 2025‑02‑10
### Added
- Initial skid‑steer motor control implementation.
  - Reads throttle (ch 2) and steering (ch 4) via FlySky IBus.
  - Maps 1000–2000 µs range to –127 to +127 speed.
  - Issues Fwd/Back commands to M1/M2 on the RoboClaw.

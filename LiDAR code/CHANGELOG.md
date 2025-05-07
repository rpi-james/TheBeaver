# Changelog

All notable changes to this LiDAR packet parser for RPLIDAR C1 are documented here.

## [0.3.0] – 2025‑MM‑DD
### Added
- Switched from per‑packet output to full‑circle array output.
- Rounded angles to integer degrees for consistent indexing.
- Coordinate transformation (polar to XY) for spatial mapping.
- Smoothed delta‑angle calculation using `lastAngle` to improve scan accuracy.
- Improved packet sync verification.

## [0.2.0] – 2025‑MM‑DD
### Changed
- Introduced `degreeDistances[360]` array for per‑degree storage.
- Populated global distance map in `loop()` using 40 measurements per packet.
- Added `noDataCount` logic: after 5 misses, input buffer is flushed to re‑sync.
- Initialized `degreeDistances` to zero in `setup()`.
- Enhanced debug loop to sequentially print all valid degree entries.

## [0.1.0] – 2025‑MM‑DD
### Added
- Initial LiDAR parsing release.
  - Initializes `Serial1` at 460800 baud to communicate with LiDAR.
  - Sends start/stop commands and verifies the 7‑byte handshake response.
  - Reads one 84‑byte scan packet and decodes 40 angle/distance pairs.
  - Outputs the first (angle, distance) measurement over USB Serial for debugging.

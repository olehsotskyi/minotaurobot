# Progress — LiDAR (Sami)

## Goal
Understand and implement reading of 2D LiDAR data on the Robot_7,
and document its topic structure and behaviour for the team.

## Execution log
- 2026-10-01: Confirmed no LiDAR topic published by default; found hardware
  via lsusb (Silicon Labs CP210x at /dev/ttyUSB0). Package rplidar_ros
  already installed.
- 2026-10-01: rplidar_a3.launch.py (256000 baud) timed out - not an A3.
  rplidar.launch.py (115200 baud, A1/A2) connected successfully.
  S/N 54F4FA89...670, firmware 1.29, hardware rev 7, health status 0.
- 2026-10-01: Topic /scan (sensor_msgs/msg/LaserScan), frame_id laser.
  angle_min=-3.124 rad, angle_max=3.142 rad, angle_increment=0.00871 rad
  (~0.5deg, ~720 points/scan), range_min=0.15m, range_max=12.0m,
  scan_time=0.122s (~8.2 Hz).
- 2026-10-01: Confirmed .inf range values correlate with intensity=0.0 -
  low return signal causes invalid range.
- 2026-10-01: Built lidar_reader package and ran lidar_subscriber node;
  confirmed front/left/right/back values respond to nearby objects.
  Placing a hand ~0.3m in front dropped `front` from ~1.5m baseline to ~0.3m.

## Next steps
- Test valid-point ratio near reflective/dark/glass surfaces.
- Decide how LiDAR data feeds into maze navigation logic.
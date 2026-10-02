# Odometry and IR reader: implementation and test log (Hermann)

My role in the team is to read the robot's odometry and IR sensor values. I started on 30 September 2026 in my own workspace (`~/hermann_ws`), first with odometry and then with the IR sensors. Both are read by one node, `odometry_ir_subscriber.py`, in my package `odometry_ir_reader`.

## Part 1: Odometry

**Goal:** read the robot's position, heading and speed.

The robot publishes its odometry on the `/robot_7/odom` topic, with the message type `nav_msgs/msg/Odometry`. This is a fused estimate: the Create 3 combines three sensors into one result.

- **Wheel encoders:** how far each wheel has turned.
- **IMU:** its gyroscope measures how fast the robot turns.
- **Optical flow sensor:** a downward-facing sensor that works like a computer mouse and measures movement over the floor.

From one odometry message we get:

- **Position** (x forward, y left, in meters from the start)
- **Heading**, stored as a quaternion, which my node converts to a yaw angle
- **Forward speed** (m/s)
- **Turning speed** (rad/s)
- **Timestamp:** when the values were measured

Commands I used:

- `ros2 topic info /robot_7/odom` shows the message type.
- `ros2 interface show nav_msgs/msg/Odometry` shows the structure of the message.
- `ros2 topic echo /robot_7/odom --once` shows one real message.

## Part 2: IR sensors

**Goal:** read the values of the robot's 7 IR sensors.

The robot publishes them on the `/robot_7/ir_intensity` topic, with the message type `irobot_create_msgs/msg/IrIntensityVector`. All 7 sensors are in the front bumper. Each reading has a `frame_id` that names the sensor (for example `ir_intensity_front_left`) and a `value`.

The values are raw intensities, not distances in meters. A value of 0 means nothing is detected, and the value rises as an obstacle gets closer. To turn them into wall distances, the team needs the calibration table in the results below.

Command I used to check information about the Topic: `ros2 topic info /robot_7/ir_intensity`.

## Implementation

- **Package:** `odometry_ir_reader` (Python, `ament_python`), created in `~/hermann_ws/src`.
- **Node:** `odometry_ir_subscriber.py`, one node with two subscriptions, one per topic. Each callback logs the latest values.
- **QoS:** both subscriptions use `qos_profile_sensor_data` (best effort), which works with the robot's sensor topics.
- **Heading:** the orientation quaternion is converted to a yaw angle with `atan2`, and logged in degrees.
- **Run:**

```bash
cd ~/hermann_ws
colcon build --packages-select odometry_ir_reader --symlink-install
source install/setup.bash
ros2 run odometry_ir_reader odometry_ir_subscriber
```

## Problems and fixes
**No module named 'odometry_ir_reader'**
Cause: __init__.py was missing from the code folder, so the build skipped my code.
Fix: created __init__.py and rebuilt the package.
**IndentationError**
Cause: the file mixed tabs and spaces.
Fix: rewrote the file with 4 spaces per level and checked it with python3 -m py_compile.
**Node started but logged no values**
Cause: the robot's topics are under the /robot_7 namespace.
Fix: changed the topic names to /robot_7/odom and /robot_7/ir_intensity.
**Workspace not connected to GitHub**
Cause: a stray, empty Git repo in the home folder.
Fix: removed it and cloned the team repo into ~/hermann_ws on my branch hermann.


## Test results
**IR Test**
## IR readings by robot orientation

Values are IR intensity, with the measured distance in brackets. The robot sits in a corner formed by two walls (90 degress); each row describes where those walls are relative to the robot.

| Orientation | Side left | Left | Front left | Front center left | Front center right | Front right | Right |
|-------------|-----------|------|------------|-------------------|--------------------|-------------|-------|
| Facing into the corner: wall ahead, wall on its left | 3900 (6 cm) / 3000 (9 cm) | 570 (11 cm) / 1050 (7 cm) | 560 (5.5 cm) / 3500 (3 cm) | 2050 (4.5 cm) / 3200 (2.5 cm) | 1800 (5.1 cm) / 3200 (3 cm) | 270 (8.5 cm) / 600 (7.5 cm) | 15 (>22 cm) / 20 (>24.5 cm) |
| Facing into the corner: wall ahead, wall on its right | 20 (>69 cm) | 750 (8.5 cm) | 1600 (3 cm) | 3700 (2.2 cm) | 3050 (3.2 cm) | 630 (9.6 cm) | 1100 (5 cm) |
| Facing away from the corner: open space ahead, wall on its right | 10 | 18 | 28 | 27 | 20 | 8 | 800 (6.2 cm) |
| Facing away from the corner: open space ahead, wall on its left | 1000 (6.2 cm) | 80 (16 cm) | 23 | 35 | 25 | 7 | 5 |

## IR readings by distance

| Distance | Side left | Left | Front left | Front center left | Front center right | Front right | Right |
|---------:|----------:|-----:|-----------:|------------------:|-------------------:|------------:|------:|
| 0 cm   | 900> | 1700> | 1700> | 2300> | 1900> | 1500> | 1400> |
| 2.5 cm | 3700> | 3800> | 3800> | 3700> | 3700> | 3700> | 3500> |
| 5 cm   | 2400> | 2300> | 1400> | 1400> | 2000> | 2000> | 1900> |
| 10 cm  | 650> | 650> | 260> | 390> | 300> | 400> | 300> |
| 15 cm  | 220> | 250> | 150> | 190> | 100> | 150> | 70> |
| 20 cm  | 100> | 120> | 90> | 100> | 50> | 50> | 20> |
| 25 cm  | 40> | 40> | 60> | 60> | 30> | 15> | 5> |

## Conclusion

The node reads and logs the robot's position, heading, speeds and all 7 IR values, and it responds correctly when the robot moves or an object comes close. Two limits remain: the IR values are raw intensities, so the calibration table is needed before they can be used as wall distances, and odometry drifts slowly over longer runs. Next, I will fill in the measured values above and hand the IR table to the Sensors role for `wall_detector`.

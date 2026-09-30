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
No module named 'odometry_ir_reader'
Cause: __init__.py was missing from the code folder, so the build skipped my code.
Fix: created __init__.py and rebuilt the package.
IndentationError
Cause: the file mixed tabs and spaces.
Fix: rewrote the file with 4 spaces per level and checked it with python3 -m py_compile.
Node started but logged no values
Cause: the robot's topics are under the /robot_7 namespace.
Fix: changed the topic names to /robot_7/odom and /robot_7/ir_intensity.
Workspace not connected to GitHub
Cause: a stray, empty Git repo in the home folder.
Fix: removed it and cloned the team repo into ~/hermann_ws on my branch hermann.


## Test results

A separate test script moved the robot while the node ran. The node logged the changing position and heading, and the IR values rose when an object was brought close to a sensor.

## Conclusion

The node reads and logs the robot's position, heading, speeds and all 7 IR values, and it responds correctly when the robot moves or an object comes close. Two limits remain: the IR values are raw intensities, so the calibration table is needed before they can be used as wall distances, and odometry drifts slowly over longer runs. Next, I will fill in the measured values above and hand the IR table to the Sensors role for `wall_detector`.

The End
## Progress Presentation Group 7
**Group members:**
- Oleh Sotskyi
- Hermann Buma Afeseh-Bidnyugha
- Sami Akasha-Mohamed
### Progress so far:
#### 1. Installed LiDAR and Raspberry Pi on Create3 Robot.
<img src="/images/Minotaurobot_1.jpeg" alt="Minotaurobot 1" width="600" height="1000">

#### 2. Can connect to the robot remotely through SSH.
<img src="images/SSH_connection.jpeg" alt="SSH connection" width="600" height="1000">

#### 3. Checked that LiDAR works with Foxglove.
<img src="images/Foxglove.jpeg" alt="Foxglove" width="900" height="500">

#### 4. Finished the ROS2 Crash Course.


### Possible approaches:
#### 1. Using advanced libraries provided by ROS2:
This approach relies on using the LiDAR to create a map of the maze first, and navigate this known map afterwards.
  - **SLAM:** Simultaneous Localization and Mapping. Allows the robot to build a map of an unknown maze and determine its location in that map at the same time.
  - **Nav2:** Navigation library that allows the robot to autonomously navigate the maze.

#### 2. Using base Create3 and ROS2 functionality with a custom algorithm:
Implementing one of the so called **Bug** algorithms would be the base of this approach. Other algorithms are being investigated.
  1. Move from starting point to goal point with a direct linear trajectory.
  2. If there is an obstacle detected with a LiDAR then avoid obstacle by making a decision to follow the obstacle wall left or right using IR sensors.
  3. When a point on the initial trajectory is reached stop avoiding the obstacle and continue moving towards the goal.

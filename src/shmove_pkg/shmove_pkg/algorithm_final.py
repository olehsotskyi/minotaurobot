"""Module providing a complete maze solving algorithm for Create3 robot with a Lidar."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

X = 2.0 # X goal coordinate
Y = 0.0 # Y goal coordinate


class PID:
    """
    Class representing a PID controller.

    Attributes:
        kp: proportional gain.
        ki: integral gain.
        kd: derivative gain.
        maximum: maximum output value for clamping it for more stable control.
        minimum: minimum output value for clamping it for more stable control.
        desired_value: desired setpoint to control the actual measured process variable.
        accumulated_error: used to calculate the integral term.
        last_error: used to calculate the derivative term.
    """

    def __init__(self, kp, ki, kd, maximum, minimum, desired_value):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maximum = maximum
        self.minimum = minimum
        self.desired_value = desired_value
        self.accumulated_error = 0.0
        self.last_error = 0.0

    def update(self, current_value, dt):
        """
        Method calculates the PID controller ouput.

        Arguments:
            current_value: actual measured process variable.
            dt: time step of the controller.

        Returns:
            output: adjusted value based on the control parameters.
        """
        error = self.desired_value - current_value
        self.accumulated_error += error * dt
        d_error = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.accumulated_error + self.kd * d_error

        # Clamp output to limits (anti-windup)
        output = max(self.minimum, min(self.maximum, output))

        self.last_error = error
        return output

    def reset(self):
        """Method resets the accumulated and last error for the next use of the controller"""
        self.accumulated_error = 0.0
        self.last_error = 0.0


class P:
    """
    Class representing a P controller.

    Attributes:
        kp: proportional gain.
        maximum: maximum output value for clamping it for more stable control.
        minimum: minimum output value for clamping it for more stable control.
        desired_value: desired setpoint to control the actual measured process variable.
    """

    def __init__(self, kp, maximum, minimum, desired_value):
        self.kp = kp
        self.maximum = maximum
        self.minimum = minimum
        self.desired_value = desired_value

    def update(self, current_value, invert=False):
        """
        Method calculates the P controller ouput.

        Arguments:
            current_value: actual measured process variable.
            invert (boolean): flag to invert the output calculation.

        Returns:
            output: adjusted value based on the control parameters.
        """
        error = self.desired_value - current_value

        if invert:
            # larger error decreases output
            output = self.maximum - self.kp * abs(error)
        else:
            # larger error increases output
            output = self.kp * error

        # Clamp output to limits (anti-windup)
        output = max(self.minimum, min(self.maximum, output))
        return output


class ShmovingController(Node):
    """Class representing the maze-solving controller ROS2 node."""

    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('ShmovingController')

        # goal coordinates
        self.x_goal = x_goal    # [m]
        self.y_goal = y_goal    # [m]

        # current robot pose
        self.x = 0.0    # [m]
        self.y = 0.0    # [m]
        self.phi = 0.0  # orientation angle [rad]

        # robot speed commands
        self.linear = 0.0   # [m/s]
        self.angular = 0.0  # [m/s]

        # IR sensor and Lidar readings
        self.ir_vals = [0.0] * 7
        self.obstacle = 'none'

        # additional control attributes
        self.last_side = 1  # saves last followed wall side
        self.side = -1  # saves side to follow wall
        self.maneuver_phi = 0.0 # variable to store orientation before maneuvering [rad]
        self.dt = 0.01          # time step for controllers and state machine loop [s]
        self.state = 'reset pose'   # current state

        # PID controller for angular speed when following wall based on wall proximity error
        self.wf_angular_pid = PID(
            kp=0.00668,
            ki=0.0,
            kd=0.0016,
            maximum=1.9,
            minimum=-1.9,
            desired_value=150
        )

        # P controller for linear speed when following wall based on wall proximity error
        self.wf_linear_p = P(
            kp=0.0004,
            maximum=0.3,
            minimum=0.01,
            desired_value=150
        )

        # PID controller for angular speed when going to goal based on orientation error
        self.gtg_angular_pid = PID(
            kp=1.5,
            ki=0.0,
            kd=0.15,
            maximum=1.9,
            minimum=-1.9,
            desired_value=0.0
        )

        # P controller for linear speed when going to goal based on orientation error
        self.gtg_linear_p = P(
            kp=1.0,
            maximum=0.3,
            minimum=0.01,
            desired_value=0.0
        )

        # P controller for angular speed when turning to goal based on orientation error
        self.ttg_angular_p = P(
            kp=1.5,
            maximum=1.9,
            minimum=-1.9,
            desired_value=0.0
        )

        # P controller for linear speed when approaching obstacle based on orientation error
        self.ao_linear_p = P(
            kp=1.5,
            maximum=0.1,
            minimum=0.01,
            desired_value=0.0
        )

        # thresholds to decide positon and orientation are reached
        self.dist_precision = 0.01
        self.phi_precision = 0.01

        # hit point / U-shape loop escape
        self.hit_point = None          # (x, y) where the robot first met the obstacle
        self.hit_distance = None       # robot-to-goal distance at the hit point
        self.threshold = None          # robot must be closer to the goal than this to leave the wall
        self.hit_point_count = 0       # how many times the robot came back to the hit point
        self.left_hit_point = False    # True once the robot has moved away from the hit point
        self.hit_radius = 0.15         # within this distance = "at the hit point"
        self.leave_radius = 0.30       # must get this far away before a revisit counts
        self.leave_margin = 0.05       # must be this much closer than threshold to leave (odom noise)
        self.min_dist_on_loop = float('inf')  # closest the robot got to the goal during the current lap

        # subscription to odometry sensors' data node
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/robot_7/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # subscription to IR sensors' data node
        self.ir_subscription = self.create_subscription(
            IrIntensityVector,
            '/robot_7/ir_intensity',
            self.ir_callback,
            qos_profile_sensor_data
        )

        # subscription to obstacle data node
        self.lidar_subscriber = self.create_subscription(
            String,
            '/robot_7/obstacle',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # create reset pose service client
        self.reset_cli = self.create_client(
            ResetPose,
            '/robot_7/reset_pose'
        )

        # wheel speeds topic publisher
        self.wheel_speed_publisher = self.create_publisher(
            Twist,
            "/robot_7/cmd_vel",
            10
        )

        # timer for calling state machine periodically
        self.timer = self.create_timer(
            self.dt,
            self.state_machine
        )

        self.get_logger().info('Started program statement')

    def lidar_callback(self, msg):
        """Method called every time lidar obstacle data is received."""
        # Only store the most recent lidar obstacle data
        self.obstacle = msg.data
        self.get_logger().info(f'Obstacle: {msg.data}', throttle_duration_sec=1)

    def ir_callback(self, msg):
        """Method called every time IR sensors data is received."""
        # Only store the most recent IR sensors' readings
        self.ir_vals = [r.value for r in msg.readings]

    def odom_callback(self, msg):
        """Method called every time odometry data is received."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientation is read as a quaternion, converted and stored as an angle [rad]
        q = msg.pose.pose.orientation
        self.phi = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def get_wall_proximity(self):
        """
        Method calculates weighted average of IR sensors readings for wall following.
        Focuses more on direct front and direct side sensors.
        Includes all sensors on respective side.
        """
        if self.side == -1: # Right side
            return (3*self.ir_vals[6] + self.ir_vals[5] + self.ir_vals[4] + 3*self.ir_vals[3])/8
        if self.side == 1:  # Left side
            return (3*self.ir_vals[0] + self.ir_vals[1] + self.ir_vals[2] + 3*self.ir_vals[3])/8

    def get_pose_error(self):
        """Method calculates distance and orientation error from current and goal coordinates."""
        x_err = self.x_goal - self.x
        y_err = self.y_goal - self.y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        phi_goal = np.arctan2(y_err,x_err)
        phi_err = phi_goal - self.phi

        # Limit the error to (-pi, pi)
        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))

        return dist_err, phi_err_correct

    def get_maneuver_error(self):
        """Method calculates orientation correction for avoiding outer corner obstacles."""
        phi_maneuver = self.maneuver_phi - (self.side * np.pi / 2.0)
        phi_err = phi_maneuver - self.phi

        # Limit the error to (-pi, pi)
        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))
        return phi_err_correct

    def check_goal(self):
        """Method checks if the goal is reached."""
        dist_err, _ = self.get_pose_error()
        return dist_err < self.dist_precision

    def check_orientation(self):
        """Method checks if robot is alligned with goal direction."""
        _, phi_err = self.get_pose_error()
        return abs(phi_err) < self.phi_precision

    def check_wall(self):
        """Method checks if the robot faces outer corners."""
        if self.side == -1:  # If wall right corner at IR sensors front and front/right
            outer_corner = [3, 4]
            free_space = [0, 1, 2, 5, 6]
        elif self.side == 1:  # If wall left corner at IR sensors front and front/left
            outer_corner = [2, 3]
            free_space = [0, 1, 4, 5, 6]
        else:
            return False

        outer_corner_vals = [self.ir_vals[i] for i in outer_corner]
        free_space_vals = [self.ir_vals[i] for i in free_space]

        # Only switch to maneuver when very close to outer corner and clear everywhere else
        if all(v > 150 for v in outer_corner_vals) and all(v < 60 for v in free_space_vals):
            low = sum(free_space_vals) / len(free_space_vals)
            high = sum(outer_corner_vals) / len(outer_corner_vals)
            return low < 50 and high > 380

        return False

    def handle_side_obstacle(self):
        self.set_wall_side()
        if self.check_wall():          # edge -> maneuver around it
            self.gtg_angular_pid.reset()
            self.state = 'maneuver'
        else:                          # wall -> follow it
            self.wf_angular_pid.reset()
            self.state = 'follow wall'

    def record_hit_point(self):
        # only record the first contact with this obstacle
        if self.hit_point is None:
            dist, _ = self.get_pose_error()
            self.hit_point = (self.x, self.y)
            self.hit_distance = float(dist)
            self.threshold = None          # no restriction until the robot is trapped
            self.min_dist_on_loop = float('inf')
            self.hit_point_count = 0
            self.left_hit_point = False
            self.get_logger().info(f'Hit point recorded at ({self.x:.2f}, {self.y:.2f}), goal distance {self.hit_distance:.2f} m')

    def check_hit_point_revisit(self):
        if self.hit_point is None:
            return
        # remember the closest the robot gets to the goal during this lap
        if self.left_hit_point:
            dist_to_goal, _ = self.get_pose_error()
            self.min_dist_on_loop = min(self.min_dist_on_loop, float(dist_to_goal))

        d = np.hypot(self.x - self.hit_point[0], self.y - self.hit_point[1])
        if not self.left_hit_point:
            if d > self.leave_radius:
                self.left_hit_point = True
        elif d < self.hit_radius:
            # robot looped back to the hit point -> it is trapped
            self.hit_point_count += 1
            if self.hit_point_count == 1:
                self.threshold = self.hit_distance   # trapped: only leave the wall once closer than the hit point
            else:
                # still looping: set the threshold just above the closest point seen on the last lap,
                # so on the next lap the robot is guaranteed to leave the wall there
                self.threshold = max(self.threshold, self.min_dist_on_loop + 2 * self.leave_margin)
            self.min_dist_on_loop = float('inf')   # start measuring the next lap
            self.left_hit_point = False
            self.get_logger().info(f'TRAPPED: hit point revisited ({self.hit_point_count}x), threshold now {self.threshold:.2f} m')

    def can_leave_wall(self):
        # normal behaviour until the robot has come back to the hit point
        if self.hit_point is None or self.hit_point_count == 0:
            return True
        if not self.left_hit_point:
            return False   # still near the hit point: don't leave here, or it drives back into the U
        dist, _ = self.get_pose_error()
        return dist < self.threshold

    def clear_hit_point(self):
        self.get_logger().info('Progress made past hit point, clearing hit point')
        self.hit_point = None
        self.hit_distance = None
        self.threshold = None
        self.min_dist_on_loop = float('inf')
        self.hit_point_count = 0
        self.left_hit_point = False

    def send_wheel_commands(self):
        """Method to create and publish a message that sets linear and angular speeds."""
        msg = Twist()

        msg.linear.x = self.linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular

        self.wheel_speed_publisher.publish(msg)

    def set_wall_side(self):
        """
        Method sets the value for angular speed control based on the wall side.
        Tracks the last side robot followed the wall on for if obstacle is directly in front.
        Sets the reference angle for maneuvering to current orientation angle.
        """
        if self.obstacle == 'right':
            self.side = -1  # right side
        elif self.obstacle == 'left':
            self.side = 1   # left side
        elif self.obstacle == 'front':
            self.side = -1 * self.last_side

        self.last_side = self.side
        self.maneuver_phi = self.phi
        self.record_hit_point()

    def reset_pose(self):
        """
        Method sets current pose as the origin at the start of the run.
        Creats and calls a request to ResetPose service.
        """
        reset_req = ResetPose.Request()
        reset_req.pose = Pose()

        reset_req.pose.position.x = 0.0
        reset_req.pose.position.y = 0.0
        reset_req.pose.position.z = 0.0

        self.reset_cli.call_async(reset_req)

    def turn_to_goal(self):
        """
        Method alligns robot with goal direction at the start of the run.
        Obtains current orintation error from a helper function to update angular speed.
        """
        self.get_logger().info('Turning', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()

        # Linear speed set to zero to turn in place
        self.linear = 0.0
        # Angular speed calculated with P controller based on orientation error so inverted=False
        # Pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        self.angular = self.ttg_angular_p.update(-phi_err)

        self.send_wheel_commands()

    def go_to_goal(self):
        """
        Method used for going in a straight line towards the goal at max available robot linear speed.
        Obtains current goal orintation error from a helper function.
        Updates linear and angular speeds.
        """
        self.get_logger().info('Going to goal', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()

        # Pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        # Linear speed calculated with P controller based on orientation error so inverted=True
        self.linear = self.gtg_linear_p.update(-phi_err, invert=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def approach_obstacle(self):
        """
        Method used to go towards the goal slower if an incoming obstacle is detected.
        Obtains current goal orintation error from a helper function.
        Updates linear and angular speeds.
        Reuses PID controller that controls angular speed when going towards goal with max speed.
        """
        self.get_logger().info('Approaching obstacle', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()

        # Pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        # Linear speed calculated with P controller based on orientation error so inverted=True
        self.linear = self.ao_linear_p.update(-phi_err, invert=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def maneuver(self):
        """
        Method used to avoid obstacles when Lidar detects them but IR sensors do not.
        Obtains orintation error for the maneuver angle from a helper function.
        Updates linear and angular speeds.
        Reuses P controller that controls linear speed when approaching obstacle.
        Reuses PID controller that controls angular speed when going towards goal with max speed.
        """
        self.get_logger().info('Maneuvering', throttle_duration_sec=1)
        phi_err = self.get_maneuver_error()

        # Pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        # Linear speed calculated with P controller based on orientation error so inverted=True
        self.linear = self.ao_linear_p.update(-phi_err, invert=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def follow_wall(self):
        """
        Method used to avoid obstacles when Lidar and IR sensors detect them by following the respective wall.
        Obtains wall proximity based on IR sensors' readings from a helper function.
        Updates linear and angular speeds.
        """
        self.get_logger().info('Following wall', throttle_duration_sec=1)
        wall_proximity = self.get_wall_proximity()

        # Linear speed calculated with P controller based on wall proximity so inverted=True
        self.linear = self.wf_linear_p.update(wall_proximity, invert=True)
        angular = self.wf_angular_pid.update(wall_proximity, self.dt)

        # Choose the angular speed sign based on what side is used for wall following
        self.angular = self.side * angular
        self.send_wheel_commands()

    def stop(self):
        """
        Method used for stopping the robot if it reached the end goal.
        Updates linear and angular speeds.
        Raises SystemExit error to shut down this controller ROS2 node.
        """
        self.linear, self.angular = 0.0, 0.0
        self.send_wheel_commands()

        self.get_logger().info('Goal reached, statement. Amaze! Amaze! Amaze!')
        raise SystemExit

    def state_machine(self):
        """
        Method used to define the state machine actions and transitions logic.
        Previously created timer keeps calling this method every dt seconds.
        """
        # Logic to handle actions in and transtions from 'reset pose' state
        if self.state == 'reset pose':
            self.reset_pose()
            self.get_logger().info(f'Goal reached: {self.check_goal()}')
            if self.check_goal():   # if goal is reached stop
                self.state = 'stop'
                self.get_logger().info('State is stop')
            elif not self.check_goal(): # if goal is not reached turn in place towards the goal
                self.state = 'turn to goal'
                self.get_logger().info('State is turn to goal')

        # Logic to handle actions in and transtions from 'turn to goal' state
        elif self.state == 'turn to goal':
            self.turn_to_goal()
            if self.check_orientation():    # if aligned with the goal direction
                if self.obstacle == 'none': # if Lidar detects no obstacles go to goal
                    self.gtg_angular_pid.reset()
                    self.state = 'go to goal'
                elif self.obstacle == 'far':   # if far obstacle detected start approaching slowly
                    self.gtg_angular_pid.reset()
                    self.state = 'approach obstacle'
                elif self.obstacle in ('left', 'right'):    # if side obstacle detected
                    self.handle_side_obstacle() # check if it is safe to wall follow with IR sensors
                elif self.obstacle == 'front':  # if obstacle is directly in front
                    self.wf_angular_pid.reset()
                    self.set_wall_side()        # set side opposite to the last one
                    self.state = 'follow wall'
            elif not self.check_orientation(): # if not alligned with goal direction keep turning
                pass

        # Logic to handle actions in and transtions from 'go to goal' state
        elif self.state == 'go to goal':
            self.go_to_goal()
            self.check_hit_point_revisit()
            if self.hit_point is not None:
                dist, _ = self.get_pose_error()
                if dist < self.hit_distance - self.leave_margin:
                    self.clear_hit_point()
            if self.check_goal():   # if goal is reached stop
                self.state = 'stop'
            elif self.obstacle == 'none':   # if lidar detects no obstacles go to goal
                pass
            elif self.obstacle == 'far':    # if lidar detects obstacle far approach slowly
                self.state = 'approach obstacle'
            elif self.obstacle in ('left', 'right'):    # if side obstacle detected
                self.handle_side_obstacle() # check if it is safe to wall follow with IR sensors
            elif self.obstacle == 'front':  # if obstacle directly in front
                self.wf_angular_pid.reset()
                self.set_wall_side()    # set side opposite to the last one
                self.state = 'follow wall'

        # Logic to handle actions in and transtions from 'approach obstacle' state
        elif self.state == 'approach obstacle':
            self.approach_obstacle()
            self.check_hit_point_revisit()
            if self.check_goal():   # if goal is reached stop
                self.state = 'stop'
            elif self.obstacle == 'none':   # if lidar detects no obstacles go to goal
                self.state = 'go to goal'
            elif self.obstacle == 'far':    # if lidar detects obstacle far approach slowly
                pass
            elif self.obstacle in ('left', 'right'):    # if side obstacle detected
                self.handle_side_obstacle() # check if it is safe to wall follow with IR sensors
            elif self.obstacle == 'front':  # if obstacle directly in front
                self.wf_angular_pid.reset()
                self.set_wall_side()    # set side opposite to the last one
                self.state = 'follow wall'

        # Logic to handle actions in and transtions from 'maneuver' state
        elif self.state == 'maneuver':
            self.maneuver()
            self.check_hit_point_revisit()
            if self.check_goal():   # if goal is reached stop
                self.state = 'stop'
            elif abs(self.get_maneuver_error()) < 0.05:   # turn complete
                # if lidar detects no obstacles and robot not trapped in U-shape go to goal
                if self.obstacle == 'none' and self.can_leave_wall():
                    self.gtg_angular_pid.reset()
                    self.state = 'go to goal'
                else:   # otherwise keep following the wall to escape the U-shaped trap
                    self.wf_angular_pid.reset()
                    self.state = 'follow wall'

        # Logic to handle actions in and transtions from 'follow wall' state
        elif self.state == 'follow wall':
            self.follow_wall()
            self.check_hit_point_revisit()
            if self.check_goal():   # if goal is reached stop
                self.state = 'stop'
            elif not self.can_leave_wall():
                pass   # not closer to the goal than the threshold yet -> keep following the wall
            elif self.obstacle == 'none':   # if lidar detects no obstacles go to goal
                self.gtg_angular_pid.reset()
                self.state = 'go to goal'
            elif self.obstacle == 'far':    # if lidar detects obstacle far approach slowly
                self.gtg_angular_pid.reset()
                self.state = 'approach obstacle'
            elif self.obstacle in ('left', 'right', 'front'): # otherwise continue following wall
                pass

        # Logic to handle actions in 'stop' state
        elif self.state == 'stop':
            self.stop()


def main(args=None):
    """Main function used to start the controller node and shut it down gracefully."""
    rclpy.init(args=args)
    node = None

    try:
        node = ShmovingController()
        rclpy.spin(node)
    except KeyboardInterrupt:   # Handle Ctrl+C gracefully
        pass
    except SystemExit:  # Handle error raised to stop robot gracefully
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

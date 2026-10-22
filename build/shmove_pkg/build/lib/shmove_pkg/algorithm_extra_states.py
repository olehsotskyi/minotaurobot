import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.srv import ResetPose

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector

X = 1.0
Y = 0.0

class PID:
    def __init__(self, kp, ki, kd, max, min, desired_value):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max
        self.min = min
        self.desired_value = desired_value
        self.accumulated_error = 0.0
        self.last_error = 0.0

    def update(self, current_value, dt):
        error = self.desired_value - current_value
        self.accumulated_error += error * dt
        d_error = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.accumulated_error + self.kd * d_error

        output = max(self.min, min(self.max, output))

        self.last_error = error
        return output

    def reset(self):
        self.accumulated_error = 0.0
        self.last_error = 0.0

class P:
    def __init__(self, kp, max, min, desired_value):
        self.kp = kp
        self.max = max
        self.min = min
        self.desired_value = desired_value

    def update(self, current_value, reversed=False):
        error = self.desired_value - current_value

        if reversed:
            output = self.max - self.kp * abs(error)
        else:
            output = self.kp * error

        output = max(self.min, min(self.max, output))
        return output

class ShmovingController(Node):
    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('ShmovingController')

        # goal coordinates
        self.x_goal = x_goal
        self.y_goal = y_goal

        # current robot pose
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        
        # robot speed commands
        self.linear = 0.0
        self.angular = 0.0

        # sensor readings
        self.ir_values = [0.0] * 7
        self.obstacle = 'none'
        
        # additional control attributes
        self.last_side = 1
        self.side = -1
        self.maneuver_phi = 0.0
        self.dt = 0.01
        self.state = 'reset pose'

        self.wf_angular_pid = PID(kp=0.00668, ki=0.0, kd=0.0016, max=1.9, min=-1.9, desired_value=150)
        self.wf_linear_p = P(kp=0.0004, max=0.3, min=0.01, desired_value=150)

        self.gtg_angular_pid = PID(kp=1.5, ki=0.0, kd=0.15, max=1.9, min=-1.9, desired_value=0.0)
        self.gtg_linear_p = P(kp=1.0, max=0.3, min=0.01, desired_value=0.0)

        self.ttg_angular_p = P(kp=1.5, max=1.9, min=-1.9, desired_value=0.0)

        self.ao_linear_p = P(kp=1.5, max=0.1, min=0.01, desired_value=0.0)

        # thresholds
        self.dist_precision = 0.01
        self.phi_precision = 0.01
        self.ir_threshold = 200.0

        # topic subscriptions
        self.odometry_subscription = self.create_subscription(Odometry, '/robot_7/odom', self.odom_callback, qos_profile_sensor_data)
        self.ir_subscription = self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.lidar_subscriber = self.create_subscription(String, '/robot_7/obstacle', self.lidar_callback, qos_profile_sensor_data)

        # create service client
        self.reset_cli = self.create_client(ResetPose, '/robot_7/reset_pose')

        # topic publishers        
        self.wheel_speed_publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)

        # timer
        self.timer = self.create_timer(self.dt, self.state_machine)

        self.get_logger().info('Started program statement')

    def lidar_callback(self, msg):
        self.obstacle = msg.data
        self.get_logger().info(f'Obstacle: {msg.data}', throttle_duration_sec=1)

    def ir_callback(self, msg):
        self.ir_values = [r.value for r in msg.readings]
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.phi = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    
    def get_wall_distance(self):
        if self.side == -1:
            current_distance = (3*self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + 3*self.ir_values[3])/8
        elif self.side == 1:
            current_distance = (3*self.ir_values[0] + self.ir_values[1] + self.ir_values[2] + 3*self.ir_values[3])/8
        return current_distance
    
    def get_pose_error(self):
        x_err = self.x_goal - self.x
        y_err = self.y_goal - self.y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        phi_goal = np.arctan2(y_err,x_err)
        phi_err = phi_goal - self.phi

        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))

        return dist_err, phi_err_correct

    def get_maneuver_error(self):
        phi_maneuver = self.maneuver_phi - (self.side * np.pi / 2.0)
        phi_err = phi_maneuver - self.phi

        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))
        return phi_err_correct
    
    def check_reset(self):
        pass
    
    def check_goal(self):
        dist_err, _ = self.get_pose_error()
        return dist_err < self.dist_precision

    def check_orientation(self):
        _, phi_err = self.get_pose_error()
        return abs(phi_err) < self.phi_precision

    def check_wall(self):
        #current_distance = self.get_wall_distance()
        #self.get_logger().info(f'Current distance: {current_distance}')
        #self.get_logger().info(f'IR sensors: {self.ir_values}')
        if self.side == -1: # wall right
            if all(value > 150 for value in self.ir_values[3:5]) and all(value < 60 for value in self.ir_values[:3]) and all(value < 60 for value in self.ir_values[5:]):
                low = (self.ir_values[0] + self.ir_values[1] + self.ir_values[2] + self.ir_values[5] + self.ir_values[6])/5
                high = (self.ir_values[3] + self.ir_values[4])/2
                return low < 40 and high > 300  
        elif self.side == 1: # wall left
            return self.ir_values[1] > 350
    
    def send_wheel_commands(self):
        msg = Twist()
        
        msg.linear.x = self.linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular

        self.wheel_speed_publisher.publish(msg)

    def set_wall_side(self):
        if self.obstacle == 'right':
            self.side = -1  # right side
        elif self.obstacle == 'left':
            self.side = 1   # left side
        elif self.obstacle == 'front':
            self.side = -1 * self.last_side
        
        self.last_side = self.side
        self.maneuver_phi = self.phi
    
    def reset_pose(self):
        reset_req = ResetPose.Request()
        reset_req.pose = Pose()

        reset_req.pose.position.x = 0.0
        reset_req.pose.position.y = 0.0
        reset_req.pose.position.z = 0.0

        self.reset_cli.call_async(reset_req)

    def turn_to_goal(self):
        self.get_logger().info(f'Turning', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()

        self.linear = 0.0
        self.angular = self.ttg_angular_p.update(-phi_err)

        self.send_wheel_commands()
    
    def go_to_goal(self):
        self.get_logger().info(f'Going to goal', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()
        
        #pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        self.linear = self.gtg_linear_p.update(-phi_err, reversed=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def approach_obstacle(self):
        self.get_logger().info(f'Approaching obstacle', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()

        #pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        self.linear = self.ao_linear_p.update(-phi_err, reversed=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def maneuver(self):
        self.get_logger().info(f'Maneuvering', throttle_duration_sec=1)
        phi_err = self.get_maneuver_error()

        #pass -phi_err so error = desired_value (0.0) - (-phi_err) = phi_err
        self.linear = self.ao_linear_p.update(-phi_err, reversed=True)
        self.angular = self.gtg_angular_pid.update(-phi_err, self.dt)

        self.send_wheel_commands()

    def follow_wall(self):
        self.get_logger().info(f'Following wall', throttle_duration_sec=1)
        wall_distance = self.get_wall_distance()
        
        self.linear = self.wf_linear_p.update(wall_distance, reversed=True)
        angular = self.wf_angular_pid.update(wall_distance, self.dt)

        self.angular = self.side * angular
        self.send_wheel_commands()

    def stop(self):
        self.linear, self.angular = 0.0, 0.0
        self.send_wheel_commands()

        self.get_logger().info('Stopped program statement')
        raise SystemExit
    
    def state_machine(self):
        if self.state == 'reset pose':
            self.reset_pose()
            self.get_logger().info(f'Goal reached: {self.check_goal()}')
            if self.check_goal():
                self.state = 'stop'
                self.get_logger().info(f'State is stop')
            elif not self.check_goal():
                self.state = 'turn to goal'
                self.get_logger().info(f'State is turn to goal')

        elif self.state == 'turn to goal':
            self.turn_to_goal()
            if self.check_orientation():
                if self.obstacle == 'none':
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.gtg_angular_pid.reset()
                    self.state = 'go to goal'
                elif self.obstacle == 'far':
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.gtg_angular_pid.reset()
                    self.state = 'approach obstacle'
                elif self.obstacle == 'right' or self.obstacle == 'left':
                    self.set_wall_side()
                    if self.check_wall():
                        #reset PID last and accumulated errors when switching to go to goal state
                        self.wf_angular_pid.reset()
                        self.state = 'follow wall'
                    else:
                        #reset PID last and accumulated errors when switching to go to goal state
                        self.gtg_angular_pid.reset()
                        self.state = 'maneuver'
                elif self.obstacle == 'front':
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.wf_angular_pid.reset()
                    self.set_wall_side()
                    self.state = 'follow wall'
            elif not self.check_orientation():
                pass
        
        elif self.state == 'go to goal':
            self.go_to_goal()
            if self.check_goal():
                self.state = 'stop'
            elif self.obstacle == 'none':
                pass
            elif self.obstacle == 'far':
                self.state = 'approach obstacle'
            elif self.obstacle == 'right' or self.obstacle == 'left':
                self.set_wall_side()
                if self.check_wall():
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.wf_angular_pid.reset()
                    self.state = 'follow wall'
                else:
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.gtg_angular_pid.reset()
                    self.state = 'maneuver'
            elif self.obstacle == 'front':
                #reset PID last and accumulated errors when switching to go to goal state
                self.wf_angular_pid.reset()
                self.set_wall_side()
                self.state = 'follow wall'

        elif self.state == 'approach obstacle':
            self.approach_obstacle()
            if self.check_goal():
                self.state = 'stop'
            elif self.obstacle == 'none':
                self.state = 'go to goal'
            elif self.obstacle == 'far':
                pass
            elif self.obstacle == 'right' or self.obstacle == 'left':
                self.set_wall_side()
                if self.check_wall():
                    self.wf_angular_pid.reset()
                    self.state = 'follow wall'
                else:
                    self.gtg_angular_pid.reset()
                    self.state = 'maneuver'
            elif self.obstacle == 'front':
                self.wf_angular_pid.reset()
                self.set_wall_side()
                self.state = 'follow wall'

        elif self.state == 'maneuver':
            self.maneuver()
            if self.check_goal():
                self.state = 'stop'
            elif self.check_wall():
                self.wf_angular_pid.reset()
                self.state = 'follow wall'
            elif self.obstacle == 'front':
                self.wf_angular_pid.reset()
                self.state = 'follow wall'
            elif self.obstacle == 'none':
                self.gtg_angular_pid.reset()
                self.state = 'go to goal'
            elif self.obstacle == 'far':
                self.gtg_angular_pid.reset()
                self.state = 'approach obstacle'
        
        elif self.state == 'follow wall':
            self.follow_wall()
            if self.check_goal():
                self.state = 'stop'
            elif self.obstacle == 'none':
                #reset PID last and accumulated errors when switching to go to goal state
                self.gtg_angular_pid.reset()
                self.state = 'go to goal'
            elif self.obstacle == 'far':
                self.gtg_angular_pid.reset()
                self.state = 'approach obstacle'
            elif self.obstacle == 'left' or self.obstacle == 'right' or self.obstacle == 'front':
                pass
        
        elif self.state == 'stop':
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ShmovingController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
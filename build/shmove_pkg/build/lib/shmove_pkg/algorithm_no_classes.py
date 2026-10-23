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
        self.dt = 0.01
        self.state = 'reset pose'

        # wall follower PID parameters
        self.wf_pid = {
            'kp linear' : 0.002,
            'kp angular' : 0.011,
            'ki angular' : 0.00015,
            'kd angular' : 0.0001,
            'last error' : 0.0,
            'accumulated error' : 0.0
        }

        # go to goal PID parameters
        self.gtg_pid = {
            'kp linear' : 1.0,
            'kp angular' : 10.0,
            'ki angular' : 0.1,
            'kd angular' : 0.1,
            'last error' : 0.0,
            'accumulated error' : 0.0
        }

        # thresholds
        self.wall_distance = 250.0
        self.dist_precision = 0.1
        self.phi_precision = 0.1
        self.max_linear = 1.0
        self.min_linear = 0.01
        self.max_angular = 2.0
        self.min_angular = -2.0

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
    
    def get_pose_error(self):
        x_err = self.x_goal - self.x
        y_err = self.y_goal - self.y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        phi_goal = np.arctan2(y_err,x_err)
        phi_err = phi_goal - self.phi

        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))

        return dist_err, phi_err_correct
    
    def get_wall_error(self):
        if self.side == -1:
            current_distance = (self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + self.ir_values[3])/4
        elif self.side == 1:
            current_distance = (self.ir_values[0] + self.ir_values[1] + self.ir_values[2] + self.ir_values[3])/4
        return self.wall_distance - current_distance
    
    def check_goal(self):
        dist_err, _ = self.get_pose_error()
        return dist_err < self.dist_precision

    def check_orientation(self):
        _, phi_err = self.get_pose_error()
        return abs(phi_err) < self.phi_precision
    
    def pid(self, pid_values, error):
        P = pid_values['kp angular'] * error
        I = pid_values['accumulated error'] + pid_values['ki angular'] * error * self.dt
        #I = max(self.min_angular, min(self.max_angular, I))
        D = pid_values['kd angular'] * (error - pid_values['last error']) / self.dt

        angular = P + I + D
        angular = max(self.min_angular, min(self.max_angular, angular))

        linear = self.max_linear - (pid_values['kp linear'] * abs(error))
        linear = max(self.min_linear, min(self.max_linear, linear))

        return linear, angular, I

    def reset_pid(self, pid_values):
        pid_values['last error'] = 0.0
        pid_values['accumulated error'] = 0.0
    
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
    
    def reset_pose(self):
        reset_req = ResetPose.Request()
        reset_req.pose = Pose()

        reset_req.pose.position.x = 0.0
        reset_req.pose.position.y = 0.0
        reset_req.pose.position.z = 0.0

        self.reset_cli.call_async(reset_req)

    def turn_to_goal(self):
        self.get_logger().info(f'Turning', throttle_duration_sec=1)
        self.linear, self.angular = 0.0, -2.0

        self.send_wheel_commands()
    
    def go_to_goal(self):
        self.get_logger().info(f'Going to goal', throttle_duration_sec=1)
        _, phi_err = self.get_pose_error()
        self.linear, self.angular, self.gtg_pid['accumulated error'] = self.pid(self.gtg_pid, phi_err)
        self.gtg_pid['last error'] = phi_err

        self.send_wheel_commands()

    def follow_wall(self):
        self.get_logger().info(f'Following wall', throttle_duration_sec=1)
        wall_err = self.get_wall_error()
        self.linear, angular, self.wf_pid['accumulated error'] = self.pid(self.wf_pid, wall_err)
        self.wf_pid['last error'] = wall_err

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
                if self.obstacle == 'none' or self.obstacle == 'far':
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.reset_pid(self.gtg_pid)
                    self.state = 'go to goal'
                elif self.obstacle == 'right' or self.obstacle == 'left' or self.obstacle == 'front':
                    #reset PID last and accumulated errors when switching to go to goal state
                    self.reset_pid(self.wf_pid)
                    self.set_wall_side()
                    self.state = 'follow wall'
            elif not self.check_orientation():
                pass
        
        elif self.state == 'go to goal':
            self.go_to_goal()
            if self.check_goal():
                self.state = 'stop'
            elif self.obstacle == 'none' or self.obstacle == 'far':
                pass
            elif self.obstacle == 'right' or self.obstacle == 'left' or self.obstacle == 'front':
                #reset PID last and accumulated errors when switching to wall follow state
                self.reset_pid(self.wf_pid)
                self.set_wall_side()
                self.state = 'follow wall'
        
        elif self.state == 'follow wall':
            self.follow_wall()
            if self.check_goal():
                self.state = 'stop'
            elif self.obstacle == 'none' or self.obstacle == 'far':
                #reset PID last and accumulated errors when switching to go to goal state
                self.reset_pid(self.gtg_pid)
                self.state = 'go to goal'
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
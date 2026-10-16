import numpy as np

import rclpy
import rclpy.node as Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

X = 0.0
Y = 0.0

class ShmovingController(Node):
    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('ShmovingController')

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
        self.ir_values = []
        self.obstacle = 'none'
        
        # additional control attributes
        self.last_side = 1
        self.dt = 0.0
        self.state = 'reset pose'

        self.wf_pid = {
            'kp linear' : 0.0,
            'kp angular' : 0.0,
            'ki angular' : 0.0,
            'kd angular' : 0.0,
            'last error' : 0.0,
            'accumulated error' : 0.0
        }

        self.gtg_pid = {
            'kp linear' : 0.0,
            'kp angular' : 0.0,
            'ki angular' : 0.0,
            'kd angular' : 0.0,
            'last error' : 0.0,
            'accumulated error' : 0.0
        }

        # thresholds
        self.wall_distance = 0.0
        self.dist_precision = 0.1

        self.odometry_subscription = self.create_subscription(Odometry, '/robot_7/odom', self.odom_callback, qos_profile_sensor_data)
        self.ir_subscription = self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.lidar_subscription = self.create_subscription()
        
        self.wheel_speed_publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)

        self.timer = self.create_timer(self.dt, self.state_machine)

    def lidar_callback(self, msg):
        self.obstacle = msg.obstacle

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
    
    def get_wall_error(self, side):
        if side == 1:
            current_distance = (self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + self.ir_values[3])/4
        elif side == -1:
            current_distance = (self.ir_values[0] + self.ir_values[1] + self.ir_values[2] + self.ir_values[3])/4
        return self.wall_distance - current_distance
    
    def check_goal(self):
        dist_err, _ = self.get_pose_error()
        return dist_err > self.dist_precision
    
    def pid(self, pid_values, error):
        P = pid_values['kp angular'] * error
        I = pid_values['accumulated error'] + pid_values['ki angular'] * error * self.dt
        D = pid_values['kd angular'] * (error - pid_values['last error']) / self.dt

        angular = P + I + D
        angular = max(-2.0, min(2.0, angular))

        linear = self.max_linear - (pid_values['kp linear'] * abs(error))
        linear = max(0.01, min(self.max_linear, linear))

        return linear, angular, I
    
    def send_wheel_commands(self):
        msg = Twist()
        
        msg.linear.x = self.linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -1*self.angular

        self.wheel_speed_publisher.publish(msg)
    
    def reset_pose(self):
        pass

    def turn_to_goal(self):
        _, phi_err = self.get_pose_error()

    
    def go_to_goal(self):
        _, phi_err = self.get_pose_error()
        linear, angular, self.gtg_pid['accumulated error'] = self.pid(self.gtg_pid, phi_err)
        self.gtg_pid['last error'] = phi_err
        return linear, angular

    def follow_wall(self, side):
        wall_err = self.get_wall_error(side)
        linear, angular, self.wf_pid['accumulated error'] = self.pid(self.wf_pid, wall_err)
        self.gtg_pid['last error'] = wall_err
        return linear, angular

    def stop(self):
        pass
    
    def state_machine(self):
        goal_reached = self.check_goal()

        if self.state == 'reset pose':
            self.reset_pose()
            if goal_reached:
                self.state = 'stop'
            elif not goal_reached:
                self.state == 'turn to goal'

        if self.state == 'turn to goal':
            self.angular = self.turn_to_goal()
            if self.obstacle == 'none':
                self.state = 'go to goal'
            elif self.obstacle == 'right':
                self.state = 'follow wall'
                side = 1 #right side
            elif self.obstacle == 'left':
                self.state = 'follow wall'
                side = -1 #left side
            elif self.obstacle == 'front':
                self.state = 'follow wall'
                side = -1 * self.last_side
        
        if self.state == 'go to goal':
            self.linear, self.angular = self.go_to_goal()
            if goal_reached:
                self.state = 'stop'
            elif self.obstacle == 'none':
                pass
            elif self.obstacle == 'right':
                self.state = 'follow wall'
                side = 1 #right side
            elif self.obstacle == 'left':
                self.state = 'follow wall'
                side = -1 #left side
            elif self.obstacle == 'front':
                self.state = 'follow wall'
                side = -1 * self.last_side
        
        if self.state == 'follow wall':
            self.linear, self.angular = self.follow_wall(side)
            if goal_reached:
                self.state = 'stop'
            elif self.obstacle == 'none':
                self.state = 'go to goal'
            elif self.obstacle == 'left' or self.obstacle == 'right' or self.obstacle == 'front':
                pass
        
        if self.state == 'stop':
            self.stop()

def main():
    pass

if __name__ == '__main__':
    main()
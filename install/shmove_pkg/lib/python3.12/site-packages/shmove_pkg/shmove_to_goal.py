import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

X = 2.0
Y = 0.0

class NavigationController(Node):
    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('NavigationController')

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        self.x_desired = x_goal
        self.y_desired = y_goal
        self.dist_precision = 0.01

        self.kp_steering = 1.5
        self.ki_steering = 0.0
        self.kd_steering = 0.0

        self.kp_throttle = 1.0
        self.max_throttle = 1.0

        self.dt = 0.01
        self.last_error = 0.0
        self.accumulated_error = 0.0

        self.odometry_subscription = self.create_subscription(Odometry, '/robot_7/odom', self.odometry_callback, qos_profile_sensor_data)

        self.wheel_speed_publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.phi = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def get_odometry_values(self):
        return self.x, self.y, self.phi

    def get_pose_error(self):
        x_current, y_current, phi_current = self.get_odometry_values()

        x_err = self.x_desired - x_current
        y_err = self.y_desired - y_current
        dist_err = np.sqrt(x_err**2 + y_err**2)

        phi_desired = np.arctan2(y_err,x_err)
        phi_err = phi_desired - phi_current

        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))

        return dist_err, phi_err_correct

    def get_pid_values(self):
        dist_err, phi_err = self.get_pose_error()

        if dist_err > self.dist_precision:
            P = self.kp_steering*dist_err
            I = self.accumulated_error + self.ki_steering*dist_err*self.dt
            D = self.kd_steering*(dist_err - self.last_error)/self.dt

            throttle_control = P + I + D
            throttle_control = max(0.01, min(0.3, throttle_control))

            # throttle_control = self.max_throttle - (self.kp_throttle * abs(phi_err))
            # throttle_control = max(0.01, min(self.max_throttle, throttle_control))
            steering_control = 0.0

            self.last_error = phi_err
            self.accumulated_error = I

            return steering_control, throttle_control
        else:
            return 0.0, 0.0

    def control_loop(self):
        steering_control, throttle_control = self.get_pid_values()

        # print(error)
        # print(steering_control)
        # print(throttle_control)

        msg = Twist()

        msg.linear.x = throttle_control
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = steering_control

        self.wheel_speed_publisher.publish(msg)
        

def main():
    rclpy.init()
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


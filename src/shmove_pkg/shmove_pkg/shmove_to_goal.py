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
Y = 2.0

class NavigationController(Node):
    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('NavigationController')

        self.x_desired = x_goal
        self.y_desired = y_goal

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        self.dt = 0.0
        self.last_error = 0.0
        self.accumulated_error = 0.0

        self.odometry_subscription = self.create_subscription(Odometry, '/robot_7/odom', self.odometry_callback, qos_profile_sensor_data)

        self.wheel_speed_publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.phi = np.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

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

    def get_pid_values(self, e, e_previous, e_accumulated):
        pass

    def control_loop(self):
        dist_err, phi_err = self.get_pose_error()
        

def main():
    rclpy.init()
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


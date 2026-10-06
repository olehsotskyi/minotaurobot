# ~/ros2_workspace/src/wall_following_pkg/wall_following_pkg/control_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.desired_distance_right = 250  # Desired distance from the right wall in meters
        # Change this to match your vehicle
        self.kp_steering = 0.011
        self.ki_steering = 0.0005
        self.kd_steering = 0.0001
        self.kp_throttle = 0.002
        self.max_throttle = 2.0

        self.last_error = 0.0
        self.integral = 0.0

        #qos_profile = QoSProfile(depth=10)

        # Publishers for steering and throttle commands
        self.wheel_speed_publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)

        # Timer to call control loop
        self.timer = self.create_timer(0.01, self.control_loop)

        # Subscribe to the Lidar Processing Node's distance data
        self.ir_subscription = self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.ir_values, qos_profile_sensor_data)
        # self.lidar_subscription = self.create_subscription(
        #     Float32,
        #     'lidar_distance',
        #     self.lidar_callback,
        #     qos_profile_sensor_data)

        self.current_distance = 260
        #self.current_distance = float('inf')

        self.get_logger().info('Control Node has been started.')

    def ir_values(self, msg):
        self.ir_values = [r.value for r in msg.readings]
        self.current_distance = (self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + self.ir_values[3])/4

    # def lidar_callback(self, msg):
    #     self.current_distance = msg.data
    
    def control_loop(self):
        # if self.current_distance == float('inf'):
        #     return

        error = self.desired_distance_right - self.current_distance
        print(error)
        self.integral += error * 0.01
        derivative = (error - self.last_error) / 0.01
        self.last_error = error

        # Steering control
        steering_control = self.kp_steering * error + self.ki_steering * self.integral + self.kd_steering * derivative
        steering_control = max(-1.0, min(2.0, steering_control))

        # steering_msg = Float32()
        # steering_msg.data = steering_control
        # self.steering_publisher.publish(steering_msg)

        # Throttle control
        throttle_control = self.max_throttle - (self.kp_throttle * abs(error))
        throttle_control = max(0.01, min(self.max_throttle, throttle_control))
        #throttle_control = 1.0  # Fixed throttle value
        
        # throttle_msg = Float32()
        # throttle_msg.data = throttle_control
        # self.throttle_publisher.publish(throttle_msg)

        msg = Twist()

        msg.linear.x = throttle_control
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -1*steering_control

        self.wheel_speed_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
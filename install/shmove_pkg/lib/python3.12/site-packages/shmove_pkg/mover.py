import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('Mover')

        self.publisher = self.create_publisher(Twist, "/robot_7/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.mover_callback)

    def mover_callback(self):
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -2.0

        self.publisher.publish(msg)

def main():
    rclpy.init()
    mover = Mover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
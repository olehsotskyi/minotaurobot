import numpy as np

import rclpy
import rclpy.node as Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.srv import ResetPose

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class ResetterNode(Node):
    def __init__(self):
        super().__init__("ResetterNode")
        self.reset_cli = self.create_client(ResetPose, '/robot_7/reset_pose')
        self.reset_pose()

    def reset_pose(self):
        reset_req = ResetPose.Request()
        reset_req.pose = Pose()
        
        reset_req.pose.position.x = 0.0
        reset_req.pose.position.y = 0.0
        reset_req.pose.position.z = 0.0

        self.reset_cli.call_async(reset_req)

def main():
    rclpy.init()
    node = Reset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
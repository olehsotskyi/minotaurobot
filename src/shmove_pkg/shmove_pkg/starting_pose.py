import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import ResetPose
from geometry_msgs.msg import Pose

class StartingPose(Node):
    def __init__(self):
        super().__init__("StartingPose")
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
    node = StartingPose()
    rclpy.spin(node)
   # node.reset_pose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
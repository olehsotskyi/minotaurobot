import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import ResetPose
from geometry_msgs.msg import Pose

class StartingPose(Node):
    def __init__(self):
        super().__init__("StartingPose")
        self.reset_cli = self.create_client(ResetPose, '/robot_7/reset_pose')

        self.reset_pose()
        self.stop()

    def reset_pose(self):
        reset_req = ResetPose.Request()
        reset_req.pose = Pose()

        reset_req.pose.position.x = 0.0
        reset_req.pose.position.y = 0.0
        reset_req.pose.position.z = 0.0

        self.reset_cli.call_async(reset_req)

    def stop(self):
        raise SystemExit
        #rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = StartingPose()
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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration

class WallFollower(Node):
    def __init__(self):
        super().__init__("WallFollower")
        self.action_client = ActionClient(self, WallFollow, '/robot_7/wall_follow')

    def send_goal(self):
        wfGoal = WallFollow.Goal()
        wfGoal.follow_side = 1

        wfGoal.max_runtime = Duration()

        wfGoal.max_runtime.sec = 300
        wfGoal.max_runtime.nanosec = 0

        self.action_client.wait_for_server()
        
        print("Publishing Goal!")

        return self.action_client.send_goal_async(wfGoal)

def main():
    rclpy.init()
    controller = WallFollower()
    controller.send_goal()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
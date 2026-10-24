import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import WallFollow
from builtin_interfaces.msg import Duration

class WallFollower(Node):
    def __init__(self):
        super().__init__("WallFollower")
        self.action_client = ActionClient(self, WallFollow, '/robot_7/wall_follow')
        self.send_goal()

    def send_goal(self):
        wfGoal = WallFollow.Goal()
        wfGoal.follow_side = 1

        wfGoal.max_runtime = Duration()

        wfGoal.max_runtime.sec = 30
        wfGoal.max_runtime.nanosec = 0

        self.action_client.wait_for_server()
        
        print("Publishing Goal!")

        self.action_client.send_goal_async(wfGoal)

def main():
    rclpy.init()
    controller = WallFollower()
    # try:
    #     controller.send_goal()
    #     rclpy.spin(controller)
    # except KeyboardInterrupt:    
    #     controller.destroy_node()
    #     rclpy.shutdown()

    # WALL_FOLLOW = True
    # while WALL_FOLLOW:
    #     try:
    #         controller.send_goal()
    #         rclpy.spin(controller)
    #     except KeyboardInterrupt:
    #         WALL_FOLLOW = False

    #controller.send_goal()
    try:
        rclpy.spin_once(controller)
    except KeyboardInterrupt:
        controller.destroy_node()
        rclpy._is_shutdown = True

if __name__ == '__main__':
    main()
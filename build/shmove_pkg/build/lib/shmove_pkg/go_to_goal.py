import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import NavigateToPosition

class ShmovingController(Node):
    def __init__(self):
        super().__init__("ShmovingController")
        
        self.action_client = ActionClient(self, NavigateToPosition, '/robot_7/navigate_to_position')

    def send_goal(self):
        shmovingGoal = NavigateToPosition.Goal()
        shmovingGoal.achieve_goal_heading = True

        shmovingGoal.goal_pose = PoseStamped()
        shmovingGoal.max_translation_speed = 1.0

        p_x = 3.0
        p_y = 3.0
        p_z = 0.0

        shmovingGoal.goal_pose.pose.position.x = p_x
        shmovingGoal.goal_pose.pose.position.y = p_y
        shmovingGoal.goal_pose.pose.position.z = p_z

        self.action_client.wait_for_server()
        
        print("Publishing Goal!")

        return self.action_client.send_goal_async(shmovingGoal)

def main():
    rclpy.init()
    controller = ShmovingController()
    controller.send_goal()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from my_custom_interfaces.msg import LidarOdom
import math


class Laser(Node):

    def __init__(self):
        super().__init__('laser_test')

        self.odometry_values = None
        
        self.last_lidar_print_time = None
        self.last_odom_print_time = None
        self.goal_x = 0.159
        self.goal_y = 1.705
        self.alpha = None
        self.beta = None 
        self.distance_to_goal = None
        self.far = None 
        self.front = None
        self.left = None
        self.right = None
        self.equal = None
        self.none = None
        
        # Obstacle lists
        self.near_obstacles = []
        self.left_distances = []
        self.right_distances = []
        self.further_obstacles = []

        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,
            '/robot_7/odom',
            self.odometry_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(
            LidarOdom, '/robot/processed_status', 10)

    def reset_scan_state(self):
        self.front = None
        self.far = None
        self.left = None
        self.right = None
        self.equal = None
        self.none = None
        self.near_obstacles = []
        self.left_distances = []
        self.right_distances = []
        self.further_obstacles = []

    def laser_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.distance_to_goal is None:
            print("Distance to goal is not yet calculated. Waiting for odometry data...")
            return

        if self.last_lidar_print_time is not None:
            if now - self.last_lidar_print_time < 1.0:
                return

        self.last_lidar_print_time = now
        print("I received a LiDAR message!")
        self.reset_scan_state()
        beta = self.beta

        # Loop: only sort each ray into the lists
        for i in range(len(msg.ranges)):
            distance = msg.ranges[i]

            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + (i * msg.angle_increment)
                angle_diff = angle - beta
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                # Zone 1: near obstacles
                if 0.18 <= distance <= 0.60 and abs(angle_diff) <= math.radians(22) and distance < self.distance_to_goal:
                    clearance = distance - 0.18
                    self.near_obstacles.append(clearance)
                    if angle_diff > 0:
                        self.left_distances.append(clearance)
                    else:
                        self.right_distances.append(clearance)

                # Zone 2: further obstacles
                elif 0.60 < distance <= 1.00 and abs(angle_diff) <= math.radians(30):
                    clearance = distance - 0.18
                    self.further_obstacles.append(clearance)

        if self.near_obstacles:
            self.front = "front"
            closest_right = min(self.right_distances) if self.right_distances else float('inf')
            closest_left = min(self.left_distances) if self.left_distances else float('inf')

            if closest_right < closest_left:
                print(f'Goal obstacle is on the RIGHT side at distance: {closest_right:.3f} m')
                self.right = 'right'
            elif closest_left < closest_right:
                print(f'Goal obstacle is on the LEFT side at distance: {closest_left:.3f} m')
                self.left = 'left'
            else:
                print(f'Goal obstacles are at equal distance on both sides: {closest_left:.3f} m')
                self.equal = 'equal'

        if self.further_obstacles:
            self.far = "far"
            print(f'Closest further obstacle distance: {min(self.further_obstacles):.3f} m')

        if not self.near_obstacles:
            self.none = "none"
            print('No obstacles detected.')

        self.lidar_odom_publishers()

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2)
        )

        speed = msg.twist.twist.linear.x
        turn = msg.twist.twist.angular.z

        self.odometry_values = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'speed': speed, 
            'turn': turn
        }

        if self.goal_x is not None and self.goal_y is not None:
            result = self.cal_dir(self.goal_x, self.goal_y)
            if result is not None:
                self.alpha = result[0] 
                self.beta = result[1]
                self.distance_to_goal = result[4]
            else:
                print("Robot is at the goal position. No direction to calculate.")

    def get_odometry_values(self):
        return self.odometry_values

    def cal_dir(self, goal_x, goal_y):
        if goal_x is None or goal_y is None:
            return None 

        x = self.odometry_values['x']
        y = self.odometry_values['y']
        phi = self.odometry_values['yaw']

        dx = goal_x - x
        dy = goal_y - y

        distance = math.hypot(dx, dy)

        if math.hypot(dx, dy) < 0.001:
            return None
            
        alpha = math.atan2(dy, dx)
        beta = alpha - phi
        beta = math.atan2(
            math.sin(beta),
            math.cos(beta)
        )

        return alpha, beta, x, y, distance

    def lidar_odom_publishers(self):

        if self.left is not None or self.right is not None or self.equal is not None:
            # Create an instance of your custom message
            out_msg = LidarOdom()

            # Assign values to the string fields in your custom message
            out_msg.left = str(self.left)
            out_msg.right = str(self.right)
            out_msg.equal = str(self.equal)
            out_msg.far = str(self.far)
            out_msg.front = str(self.front)
            out_msg.none = str(self.none)

            # Publish using the actual publisher object
            self.publisher_.publish(out_msg)

            self.get_logger().info(
                f'Published -> Left: {out_msg.left}, Right: {out_msg.right}, Equal: {out_msg.equal}, Far: {out_msg.far}, Front: {out_msg.front}, none: {out_msg.none}'
            )

def main(args=None):
    rclpy.init(args=args)
    laser = Laser()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

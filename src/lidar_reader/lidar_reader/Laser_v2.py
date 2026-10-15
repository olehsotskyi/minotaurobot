import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math


class Laser(Node):

    def __init__(self):
        super().__init__('laser_test')

        self.odometry_values = None
        
        self.last_lidar_print_time = None
        self.last_odom_print_time = None
        self.goal_x = -1.8
        self.goal_y = 0.13
        
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

    def laser_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_lidar_print_time is not None:
            if now - self.last_lidar_print_time < 1.0:
                return

        self.last_lidar_print_time = now
        print("I received a LiDAR message!")

        # 1. CRITICAL: Clear arrays at the start of every scan so old data doesn't pile up
        self.near_obstacles = []
        self.left_distances = []
        self.right_distances = []
        self.further_obstacles = []

        closest_distance = float('inf')
        closest_index = -1

        # 2. Map through all LiDAR distance readings
        for i in range(len(msg.ranges)):
            distance = msg.ranges[i]

            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                
                # Track overall closest point for reference
                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i

                # Calculate angle and Cartesian coordinates FOR EVERY VALID RAY
                angle = msg.angle_min + (i * msg.angle_increment)
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                # Zone 1: Near obstacles check
                if 0.18 <= x <= 0.60 and abs(y) <= 0.2195:
                    clearance = x - 0.18
                    self.near_obstacles.append(clearance)
                
                    # Check if it's also in the 30-degree wedge for left/right decision
                    if abs(y) <= x * math.tan(math.radians(30)):
                        if y > 0:
                            self.left_distances.append(clearance)
                        else:
                            self.right_distances.append(clearance)
                        closest_right = min(self.right_distances) if self.right_distances else float('inf')
                        closest_left = min(self.left_distances) if self.left_distances else float('inf')
                
                        if(closest_right < closest_left):
                            print(f'Closest obstacle is on the RIGHT at distance: {closest_right:.3f} m')
                        elif(closest_left < closest_right):
                            print(f'Closest obstacle is on the LEFT at distance: {closest_left:.3f} m')
                        elif(closest_left == closest_right):
                            print(f'Closest obstacles are at equal distance on both sides: {closest_left:.3f} m')

                # Zone 2: Further obstacles check
                elif 0.60 < x <= 1.00 and abs(y) <= 0.2195:
                    clearance = x - 0.18
                    self.further_obstacles.append(clearance)
                    print(f'Closest further obstacle distance: {min(self.further_obstacles):.3f} m' if self.further_obstacles else 'No further obstacles detected.')

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

        result = self.cal_dir(self.goal_x, self.goal_y)

    def get_odometry_values(self):
        return self.odometry_values

    def cal_dir(self, goal_x, goal_y):
        if goal_x is None and goal_y is None:
            return None 

        x = self.odometry_values['x']
        y = self.odometry_values['y']
        phi = self.odometry_values['yaw']

        dx = goal_x - x
        dy = goal_y - y

        if math.hypot(dx, dy) < 0.001:
            return None
            
        alpha = math.atan2(dy, dx)
        beta = alpha - phi
        beta = math.atan2(
            math.sin(beta),
            math.cos(beta)
        )

        return alpha, beta

def main(args=None):
    rclpy.init(args=args)
    laser = Laser()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
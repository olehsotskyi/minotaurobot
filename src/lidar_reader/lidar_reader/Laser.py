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
        self.goal_x = None
        self.goal_y = None

        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
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

        print("Number of measurements:", len(msg.ranges))
        print("Angle minimum:", msg.angle_min)
        print("Angle maximum:", msg.angle_max)
        print("Angle increment:", msg.angle_increment)
    
        closest_distance = float('inf')
        closest_index = -1

        for i in range(len(msg.ranges)):

            distance = msg.ranges[i]

            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:

                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i

        if closest_index != -1:

            closest_angle = (
                msg.angle_min +
                closest_index * msg.angle_increment
            )

            closest_angle_degrees = closest_angle * 180 / math.pi
            closest_x = closest_distance * math.cos(closest_angle)
            closest_y = closest_distance * math.sin(closest_angle)

            print("OBSTACLE X:", round(closest_x, 3), "m")
            print("OBSTACLE Y:", round(closest_y, 3), "m")

            if closest_y > 0:
                print("OBSTACLE SIDE: LEFT")
            elif closest_y < 0:
                print("OBSTACLE SIDE: RIGHT")
            else:
                print("OBSTACLE SIDE: CENTER")

            print(
                "CLOSEST:",
                "index:", closest_index,
                "angle:", closest_angle_degrees,
                "degrees",
                "distance:", closest_distance,
                "m"
            )


        front_distances = []
        bumper_distances = []
        triangle_distances = []
        rectangle_distances = []
        
        for i in range(len(msg.ranges)):

            angle = msg.angle_min + i * msg.angle_increment
            angle_degrees = angle * 180 / math.pi

            distance = msg.ranges[i]

            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:

                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                if 0.18 <= x <= 0.60 and abs(y) <= 0.2195:
                    

                    front_distances.append(distance)
                    bumper_distances.append(x - 0.18)

                if 0 <= x <= 0.60:

                    if abs(y) <= x * math.tan(math.radians(30)):
                        triangle_distances.append(distance)


                if 0.60 < x <= 1.00:

                    if abs(y) <= 0.2195:
                        rectangle_distances.append(distance)         

        if front_distances:

            closest_front = min(front_distances)

            print(
                "FRONT CLOSEST:",
                closest_front,
                "m"
            )

        if bumper_distances:

            closest_bumper = min(bumper_distances)

            print(
                "DISTANCE FROM FRONT BUMPER:",
                round(closest_bumper, 3),
                "m"
            )
        print("POINTS INSIDE FOCUS:", len(front_distances))
        print("TRIANGLE POINTS:", len(triangle_distances))
        print("RECTANGLE POINTS:", len(rectangle_distances))

        if triangle_distances:
            closest_triangle = min(triangle_distances)
            print("TRIANGLE: OBSTACLE DETECTED", round(closest_triangle, 3), "m")
        else:
            print("TRIANGLE: NO VALID RETURN")

        if rectangle_distances:
            closest_rectangle = min(rectangle_distances)
            print("RECTANGLE: OBSTACLE DETECTED", round(closest_rectangle, 3), "m")
        else:
            print("RECTANGLE: NO VALID RETURN")

        if front_distances:
            print("FOCUS AREA: OBSTACLE DETECTED")
        else:
            print("FOCUS AREA: CLEAR")

        right_distances = []
        left_distances = []

        robot_width = 0.339
        safety_margin = 0.05

        threshold = robot_width / 2 + safety_margin
        print("CALCULATED WALL TARGET:", round(threshold, 4), "m")

        for i in range(len(msg.ranges)):

            angle = msg.angle_min + i * msg.angle_increment
            angle_degrees = angle * 180 / math.pi

            distance = msg.ranges[i]

            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:

                y = distance * math.sin(angle)

                if -100 <= angle_degrees <= -80:
                    right_distances.append(abs(y))

                if 80 <= angle_degrees <= 100:
                    left_distances.append(y)

        if right_distances:

            closest_right = min(right_distances)

            print(
                "RIGHT CLOSEST:",
                closest_right,
                "m"
            )

        if left_distances:

            closest_left = min(left_distances)

            print(
                "LEFT CLOSEST:",
                closest_left,
                "m"
            )

            error = threshold - closest_left

            print("LEFT WALL ERROR:", round(error, 3), "m")

            if closest_left < threshold:
                print("LEFT WALL: TOO CLOSE")
            elif closest_left > threshold:
                print("LEFT WALL: FARTHER THAN TARGET")
            else:
                print("LEFT WALL: AT TARGET") 
    
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

        if self.goal_x is None:
            self.goal_x = -1.8  # X coordinate of Point B (metres)
            self.goal_y = 0.13  # Y coordinate of Point B (metres)

        result = self.cal_dir(self.goal_x, self.goal_y)

       
        now = self.get_clock().now().nanoseconds / 1e9

        if (self.last_odom_print_time is None or
                now - self.last_odom_print_time >= 1.0):

            self.last_odom_print_time = now

            self.get_logger().info(
                f'ODOM x={x:.3f} m  y={y:.3f} m  '
                f'yaw={math.degrees(yaw):.1f} deg  '
                f'speed={speed:.2f} m/s  '
                f'turn={turn:.2f} rad/s'
            )

            print(
                "TEST GOAL:",
                round(self.goal_x, 3),
                round(self.goal_y, 3)
            )

            if result is not None:
                alpha, beta = result

                distance_to_goal = math.hypot(
                    self.goal_x - x,
                    self.goal_y - y
                )

                print(
                    "DISTANCE TO GOAL:",
                    round(distance_to_goal, 3), "m"
                )

                print(
                    "GOAL DIRECTION ALPHA:",
                    round(math.degrees(alpha), 2), "degrees"
                )

                print(
                    "TURNING ERROR BETA:",
                    round(math.degrees(beta), 2), "degrees"
                )

   
    def get_odometry_values(self):
        return self.odometry_values


    def cal_dir(self, goal_x, goal_y):

        odom = self.get_odometry_values()

        if odom is None:
            return None

        x = odom['x']
        y = odom['y']
        phi = odom['yaw']

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

    try:
        rclpy.spin(laser)
    except KeyboardInterrupt:
        pass
    finally:
        laser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

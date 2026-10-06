import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class Laser(Node):

    def __init__(self):
        super().__init__('laser_test')

        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

    def laser_callback(self, msg):

        print("I received a LiDAR message!")

        print("Number of measurements:", len(msg.ranges))
        print("Angle minimum:", msg.angle_min)
        print("Angle maximum:", msg.angle_max)
        print("Angle increment:", msg.angle_increment)

   
        for i in range(0, len(msg.ranges), 20):

            angle = msg.angle_min + i * msg.angle_increment
            angle_degrees = angle * 180 / math.pi
            distance = msg.ranges[i]

            print(
                "index:", i,
                "angle:", angle_degrees,
                "degrees",
                "distance:", distance
            )

    
        closest_distance = float('inf')
        closest_index = -1

        for i in range(len(msg.ranges)):

            distance = msg.ranges[i]

            if math.isfinite(distance):

                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i

        if closest_index != -1:

            closest_angle = (
                msg.angle_min +
                closest_index * msg.angle_increment
            )

            closest_angle_degrees = closest_angle * 180 / math.pi

            print(
                "CLOSEST:",
                "index:", closest_index,
                "angle:", closest_angle_degrees,
                "degrees",
                "distance:", closest_distance,
                "m"
            )


        front_distances = []

        for i in range(len(msg.ranges)):

            angle = msg.angle_min + i * msg.angle_increment
            angle_degrees = angle * 180 / math.pi

            distance = msg.ranges[i]

            if -30 <= angle_degrees <= 30:

                if math.isfinite(distance):
                    front_distances.append(distance)

        if front_distances:

            closest_front = min(front_distances)

            print(
                "FRONT CLOSEST:",
                closest_front,
                "m"
            )

        right_distances = []
        left_distances = []

        for i in range(len(msg.ranges)):

            angle = msg.angle_min + i * msg.angle_increment
            angle_degrees = angle * 180 / math.pi

            distance = msg.ranges[i]

            if -120 <= angle_degrees <= -30:

                if math.isfinite(distance):
                    right_distances.append(distance)

            if 30 <= angle_degrees <= 120:

                if math.isfinite(distance):
                    left_distances.append(distance)

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


def main(args=None):

    rclpy.init(args=args)

    laser = Laser()

    rclpy.spin(laser)

    laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

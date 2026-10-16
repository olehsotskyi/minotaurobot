import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data
        )
        self.get_logger().info('Lidar Subscriber has been started.')

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)

        def index_for_angle(deg):
            rad = math.radians(deg)
            i = round((rad - msg.angle_min) / msg.angle_increment)
            return i % n

        front = msg.ranges[index_for_angle(0)]
        left = msg.ranges[index_for_angle(90)]
        right = msg.ranges[index_for_angle(-90)]
        back = msg.ranges[index_for_angle(180)]

        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        closest = min(valid) if valid else float('nan')

        self.get_logger().info(
            f'points={n}  valid={len(valid)}/{n}  '
            f'front={front:.2f}m  left={left:.2f}m  right={right:.2f}m  back={back:.2f}m  '
            f'closest={closest:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
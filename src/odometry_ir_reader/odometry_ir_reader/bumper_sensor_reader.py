import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection

class BumperSensorReader(Node):
    def __init__(self):
        super().__init__('bumper_sensor_reader')
        self.bumper_values = []
        self.create_subscription(HazardDetectionVector, '/robot_7/hazard_detection', self.on_bumper, qos_profile_sensor_data)
        self.get_logger().info('Bumper Sensor Reader has been started.')
    def on_bumper(self, msg):
        self.bumper_values = [
            {r.header.frame_id.replace('hazard_detection_', ''): r.type}
            for r in msg.detections
            if r.type == HazardDetection.BUMP
        ]
        self.get_logger().info('Bumper    ' + '  '.join([f"{k}={v}" for d in self.bumper_values for k, v in d.items()]))
    def get_bumper_values(self):
        return list(self.bumper_values)

def main():
    rclpy.init()
    node = BumperSensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
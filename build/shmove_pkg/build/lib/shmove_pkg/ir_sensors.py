import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import IrIntensityVector

class IrSensors(Node):
    def __init__(self):
        super().__init__('IrSensors')

        self.wall_distance = 150

        self.ir_subscription = self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.ir_values, qos_profile_sensor_data)

    def ir_values(self, msg):
        self.ir_values = [r.value for r in msg.readings]
        self.get_logger().info(f'IR sensor values: {self.ir_values}', throttle_duration_sec=1)

        self.current_distance = (self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + self.ir_values[3])/4
        error = self.wall_distance - self.current_distance
        self.get_logger().info(f'Current distance: {self.current_distance}, Error: {error}', throttle_duration_sec=1)
    
def main(args=None):
    rclpy.init(args=args)
    node = IrSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
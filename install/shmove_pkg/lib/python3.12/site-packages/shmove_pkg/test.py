import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector

import numpy as np

class WallShmover(Node):
    def __init__(self):
        super().__init__("WallShmover")

        self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.ir_callback, qos_profile_sensor_data)

        self.ir_values = []

        # self.target_distance = 0.5
        self.current_distance = 200
        
        # self.kp = 0.0
        # self.ki = 0.0
        # self.kd = 0.0
        # self.previous_error = 0.0
        # self.integral = 0.0

        self.timer = self.create_timer(1, self.printer)

    def ir_callback(self, msg):
        self.ir_values = [r.value for r in msg.readings]
        self.current_distance = (self.ir_values[6] + self.ir_values[3])/2

    def printer(self):
        print(self.current_distance)

    # def control_loop(self):
    #     error = self.

    
def main():
    rclpy.init()
    wall_shmover = WallShmover()
    rclpy.spin(wall_shmover)
    wall_shmover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
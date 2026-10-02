import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector


class OdometryIRSubscriber(Node):
    def __init__(self):
        super().__init__('odometry_ir_subscriber')
        #Intialization of an empty arry and empty dict file to hold the ir values and the odometry variables
        self.ir_values = []
        self.odometry_values = {}  
        # Create the subscriptions: message type, topic name, callback, and QoS profile
        self.create_subscription(Odometry, '/robot_7/odom', self.odometry_callback, qos_profile_sensor_data)
        self.create_subscription(IrIntensityVector, '/robot_7/ir_intensity', self.on_ir, qos_profile_sensor_data)
        self.get_logger().info('Odometry IR Subscriber has been started.')

    def odometry_callback(self, msg):
        """Called every time an /odom message arrives."""
        x = msg.pose.pose.position.x          # meters forward
        y = msg.pose.pose.position.y          # meters left
        q = msg.pose.pose.orientation         # heading as a quaternion
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        speed = msg.twist.twist.linear.x      # forward speed (m/s)
        turn = msg.twist.twist.angular.z      # turning speed (rad/s)


        #Assigning the odeomtry values to the dictionary to be returned in the getter function
        self.odometry_values = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'speed': speed,
            'turn': turn
        }

        # self.get_logger().info(
        #     f'ODOM  x={x:.3f} m  ={y:.3f} m  yaw={math.degrees(yaw):.1f} deg  '
        #     f'speed={speed:.2f} m/s  turn={turn:.2f} rad/s'
        # )
    def get_odometry_values(self):
        return self.odometry_values

    def on_ir(self, msg):
        """Called every time an /ir_intensity message arrives."""
        self.ir_values = [
            f"{r.header.frame_id.replace('ir_intensity_', '')}={r.value}"
            for r in msg.readings
        ]
        self.get_logger().info('IR    ' + '  '.join(self.ir_values))
        ## Getter Funtion to get the IR Values Array
    def get_ir_values(self):
        return list(self.ir_values)

def main():
    rclpy.init()
    node = OdometryIRSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import numpy as np

import rclpy
import rclpy.node as Node

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

X = 0.0
Y = 0.0

class ShmovingController(Node):
    def __init__(self, x_goal=X, y_goal=Y):
        super().__init__('ShmovingController')

        self.x_goal = x_goal
        self.y_goal = y_goal

        self.wall_distance = 0.0

        self.state = 'reset pose'

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        
        self.linear = 0.0
        self.angular = 0.0

        self.ir_values = []
        
        self.last_side = 1

        self.wf_pid = {
            'kp linear' = 0.0,
            'kp angular' = 0.0,
            'ki angular' = 0.0,
            'kd angular' = 0.0
            'last error' = 0.0,
            'accumulated error' = 0.0,
            'dt' = 0.0
        }

        self.gtg_pid = {
            'kp linear' = 0.0,
            'kp angular' = 0.0,
            'ki angular' = 0.0,
            'kd angular' = 0.0,
            'last error' = 0.0,
            'accumulated error' = 0.0,
            'dt' = 0.0
        }

        self.dist_precision = 0.1

    def lidar_callback(self, msg):
        pass
    
    def get_lidar_data(self):
        pass

    def ir_callback(self, msg):
        self.ir_values = [r.value for r in msg.readings]

    def get_ir_data(self):
        pass
    
    def odom_callback(self, msg):
        pass

    def get_odom_data(self):
        pass
    
    def get_pose_error(self):
        x_err = self.x_goal - self.x
        y_err = self.y_goal - self.y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        phi_goal = np.arctan2(y_err,x_err)
        phi_err = phi_goal - self.phi

        phi_err_correct = np.arctan2(np.sin(phi_err), np.cos(phi_err))

        return dist_err, phi_err_correct
    
    def get_wall_error(self):
        current_distance = (self.ir_values[6] + self.ir_values[5] + self.ir_values[4] + self.ir_values[3])/4
        return self.wall_distance - current_distance
    
    def check_goal(self):
        dist_err, _ = self.get_pose_error()
        return dist_err > self.dist_precision
    
    def detect_obstacle(self):
        pass
    
    def pid(self, pid_values, error):
        P = pid_values['kp angular'] * error
        I = pid_values['accumulated error'] + pid_values['ki angular'] * error * pid_values['dt']
        D = pid_values['kd angular'] * (error - pid_values['last error']) / pid_values['dt']

        angular = P + I + D
        angular = max(-2.0, min(2.0, angular))

        linear = self.max_linear - (pid_values['kp linear'] * abs(error))
        linear = max(0.01, min(self.max_linear, linear))

        pid_values['last error'] = error
        pid_values['accumulated error'] = I

        return linear, angular, I
    
    def send_wheel_commands(self):
        pass
    
    def reset_pose(self):
        pass
    
    def go_to_goal(self):
        _, phi_err = self.get_pose_error()
        linear, angular, self.gtg_pid['accumulated error'] = self.pid(self.gtg_pid, phi_err)
        self.gtg_pid['last error'] = phi_err
        return linear, angular

    def follow_wall(self, side):
        wall_err = self.get_wall_error()
        linear, angular, self.wf_pid['accumulated error'] = self.pid(self.wf_pid, wall_err)
        self.gtg_pid['last error'] = wall_err
        return linear, angular

    def stop(self):
        pass
    
    def state_machine(self):
        goal_reached = self.check_goal()
        obstacle = self.get_lidar_data()

        if self.state == 'reset pose':
            self.reset_pose()
            if goal_reached:
                self.state = 'stop'
            elif not goal_reached:
                self.state == 'go to goal'
        
        if self.state == 'go to goal':
            self.linear, self.angular = self.go_to_goal()
            if goal_reached:
                self.state = 'stop'
            elif obstacle == 'none':
                pass
            elif obstacle == 'right':
                self.state = 'follow wall'
                side = 1 #right side
            elif obstacle == 'left':
                self.state = 'follow wall'
                side = -1 #left side
            elif obstacle == 'front':
                state = 'follow wall'
                side = -1 * self.last_side
        
        if self.state == 'follow wall':
            self.linear, self.angular = self.follow_wall(side)
            if goal_reached:
                self.state = 'stop'
            elif obstacle == 'none':
                self.state = 'go to goal'
            elif obstacle == 'left' or obstacle == 'right' or obstacle == 'front':
                pass
        
        if state == 'stop':
            self.stop()

def main():
    pass

if __name__ == '__main__':
    main()
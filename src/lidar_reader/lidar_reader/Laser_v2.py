import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from my_custom_interfaces.msg import LidarOdom
from std_msgs.msg import String
import math
import subprocess
import time

class Laser(Node):

    def __init__(self):
        super().__init__('laser_test')

        self.odometry_values = None # initializing the odoemtry variable to none
        
        self.last_lidar_print_time = None
        self.last_odom_print_time = None

        # goal position in the odometry frame (must match the controller's goal)
        self.goal_x = 2.28
        self.goal_y = -3.82

       
        self.alpha = None    # alpha: goal direction in the world frame
        self.beta = None    # beta: Angle the robot must trun to be in the direction of the goal
        self.distance_to_goal = None    #Distance from the lidar to the goal
        self.far = None  # obstacles in the far range of the lidar from above 30cm to 360cm 
        self.front = None #Obstacles in front of the robot from 18cm to 30cm
        self.left = None  #obstacles in front of the robot from 18cm to 30cm but to the left
        self.right = None  #obstacles in front of the robot from 18cm to 30cm but to the right
        # self.equal = None   
        self.none = None  # No close obstcale
        
        # Obstacle lists
        self.near_obstacles = []  # Array holdiing Near obstacle values readings from the lidar 
        self.left_distances = []  # Array holdiing Near Left obstacle values readings from the lidar 
        self.right_distances = []   # Array holdiing Near Right obstacle values readings from the lidar 
        self.further_obstacles = []  # Array holdiing further obstacle values readings from the lidar 

        #Intiliazation of the scan subscrition to the scan topic
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        #Init of the Odoemtry Subscription from the odom topic
        self.create_subscription(
            Odometry,
            '/robot_7/odom',
            self.odometry_callback,
            qos_profile_sensor_data
        )
        #Intilization of the publisher for the cutom interface we are publishing for the alogorithm based on the lidar and odom values
        self.publisher_ = self.create_publisher(
            String, '/robot_7/obstacle', 10)

    #Reseting the variables 
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

    # runs on every lidar scan: sorts the scan points into zones around the goal direction
    def laser_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        # can't look toward the goal until odometry has given us the goal direction
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

           # ignore invalid readings (inf/NaN or outside the sensor's range)
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:

                # angle of this ray, and how far it is from the goal direction (wrapped to -pi..pi) i.e left from the center if the robot and right form the center of the robot 
                angle = msg.angle_min + (i * msg.angle_increment)
                angle_diff = angle - beta
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                # Zone 1: Reading values of near osbtacles that is from the bumper of the robot to 30cm from the lidar and + and - 45 degrees from the center of the imaginary line that cuts through the lidar 
                if 0.18 <= distance <= 0.30 and abs(angle_diff) <= math.radians(45) and distance < self.distance_to_goal:
                    clearance = distance - 0.18 #removing the distnce from the middle of the lidar to the distance
                    self.near_obstacles.append(clearance)
                    #Checking if the obstacle is to the left of to the right and inserting the lidar readings into their various arrays 
                    if angle_diff > 0:
                        self.left_distances.append(clearance)
                    else:
                        self.right_distances.append(clearance)

                # Zone 2: Reading further obstacles that is above 30cm to 60 cm 
                elif 0.30 < distance <= 0.60 and abs(angle_diff) <= math.radians(30):
                    clearance = distance - 0.18
                    self.further_obstacles.append(clearance)

        # Determing the direction of our obstacle and returning a string in their respetive variables
        if self.near_obstacles:
            closest_right = min(self.right_distances) if self.right_distances else float('inf')
            closest_left = min(self.left_distances) if self.left_distances else float('inf')

            if closest_right < closest_left:
                self.right = 'right'
            elif closest_left < closest_right:
                self.left = 'left'
            else:
                self.front = 'front'

        # Returning far obstcale in the far variable
        if self.further_obstacles:
            self.far = "far"
        #Setting no obstacle of there is no obstacle near 
        if not self.near_obstacles:
            self.none = "none"
            print('No obstacles detected.')

        self.lidar_odom_publishers()

    # Odometry callback to get the robots postion and the robots orientation 
    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x  
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

         # convert the orientation quaternion to yaw (heading angle)
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2)
        )

        speed = msg.twist.twist.linear.x
        turn = msg.twist.twist.angular.z

        # Storing the values of our odometry to the odometry variable
        self.odometry_values = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'speed': speed, 
            'turn': turn
        }  

        if self.goal_x is not None and self.goal_y is not None:
            # Calling the cal dir function which returns alpha, beta, x, y, distance
            result = self.cal_dir(self.goal_x, self.goal_y)
            if result is not None:
                self.alpha = result[0] 
                self.beta = result[1]
                self.distance_to_goal = result[4]
            else:
                print("Robot is at the goal position. No direction to calculate.")

    def get_odometry_values(self):
        return self.odometry_values

    # Funtion to calculate the direction to the goal in the world frame (Alpha) and the angle the robot has to turn to face the goal (Beta)
    def cal_dir(self, goal_x, goal_y):
        if goal_x is None or goal_y is None:
            return None 

        x = self.odometry_values['x']
        y = self.odometry_values['y']
        phi = self.odometry_values['yaw']

        # Calculating the difference between the robots odoma nd the goal in x and y
        dx = goal_x - x
        dy = goal_y - y

        # straight-line distance to the goal
        distance = math.hypot(dx, dy)

        if math.hypot(dx, dy) < 0.001:
            return None
            
        alpha = math.atan2(dy, dx) # Calculating alpha from the dif of the odom and the goal in the x and y 
        beta = alpha - phi   # beta: goal direction relative to the robot's heading, wrapped to -pi..pi
        beta = math.atan2(
            math.sin(beta),
            math.cos(beta)
        ) 

        return alpha, beta, x, y, distance


    # Lidar publishes the direction of the obstacle to our custom topic robot_7/obtscle
    def lidar_odom_publishers(self):
        print("I am runnung this")
        if self.left is not None:
            direction = 'left'

        elif self.right is not None:
            direction = 'right'
          
        elif self.front is not None:
            direction = 'front'
          
        elif self.far is not None:
            direction = 'far'
        else:
            direction = 'none'
            distance = float('inf')

        out_msg = String()
        out_msg.data = direction
        self.publisher_.publish(out_msg)
        self.get_logger().info(f'Published -> {out_msg.data}')

#Lidar initialization
def start_rplidar():
    cmd = [
        'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
        '--ros-args',
        '-p', 'serial_port:=/dev/ttyUSB0',
        '-p', 'serial_baudrate:=115200',   # A1/A2M8; 256000 for A2M12/A3, 460800 for C1
        '-p', 'frame_id:=laser',
        '-p', 'angle_compensate:=true',
    ]
    proc = subprocess.Popen(cmd)
    time.sleep(2.0)   # give the lidar time to spin up
    return proc


def main(args=None):
    lidar_proc = start_rplidar()
    rclpy.init(args=args)
    laser = Laser()
    rclpy.spin(laser)
    laser.destroy_node()
    rclpy.shutdown()
    lidar_proc.terminate()   # stop the lidar when this node stops
    lidar_proc.wait()


if __name__ == '__main__':
    main()

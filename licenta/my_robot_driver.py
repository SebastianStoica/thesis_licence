import rclpy
from geometry_msgs.msg import Twist,PointStamped
from math import atan2
import numpy as np
from sensor_msgs.msg import LaserScan
#from nav2_msgs import path


from math import fmod
import math

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
    
        self.timeStep = 1
        self.__left_motor = self.robot.getDevice('left wheel motor')
        self.__right_motor = self.robot.getDevice('right wheel motor')
        try:
            self.compass = self.robot.getDevice('compass')
            self.compass.enable(1)
            self.gps = self.robot.getDevice('gps')
        except Exception as e: 
            print(e)
        self.max_motor_velocity = 19.1
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.prev_err_lin = 0
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        self.__target_twist = Twist()
        self.__target_gps = PointStamped()
        self.lidar_data = LaserScan()
        self.goal_position = (1,0.5,1.5)
        self.max_linear_velocity = 9.0
        self.max_angular_velocity = 0.5
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.__node.create_subscription(PointStamped, 'gps_sensor', self.gps_sensor_callback, 10)
        self.__node.create_subscription(LaserScan, 'lidar_sensor', self.lidar_sensor_callback, 10)

        self.__node.get_logger().info("Start thesis")
      
    def cmd_vel_callback(self, twist):
        self.__target_twist = twist
        
    def gps_sensor_callback(self,msg):
        self.__target_gps = msg

    def lidar_sensor_callback(self,msg):
        self.lidar_data = msg


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
    
        north = self.compass.getValues()
        angle = atan2(north[1], north[0])
        theta = np.degrees(angle)

        x_pose = self.__target_gps.point.x
        y_pose = self.__target_gps.point.y

        current_position = (x_pose, y_pose, theta)
        print(f"khepera position is: {current_position}")
        distance_x = self.goal_position[0] - current_position[0]
        distance_y = self.goal_position[1] - current_position[1]
        distance = np.sqrt(distance_x ** 2 + distance_y ** 2)

        desired_ang = np.degrees(atan2(distance_y, distance_x))
        print(desired_ang)
        theta_difference = fmod(desired_ang - current_position[2] + 180, 360) - 180
        theta_error = np.degrees(atan2(math.sin(np.radians(theta_difference)), math.cos(np.radians(theta_difference))))
        print(f"theta err is : {theta_error}, theta diff is: {theta_difference}")
      
        lin_vel = distance
        ang_vel = theta_error

   
        lin_vel = min(lin_vel, self.max_linear_velocity)
        ang_vel = min(ang_vel, self.max_angular_velocity)

        left_motor_velocity = lin_vel + ang_vel
        right_motor_velocity = lin_vel - ang_vel

        left_motor_velocity = min(max(left_motor_velocity, -self.max_motor_velocity), self.max_motor_velocity)
        right_motor_velocity = min(max(right_motor_velocity, -self.max_motor_velocity), self.max_motor_velocity)

        self.__left_motor.setVelocity(left_motor_velocity)
        self.__right_motor.setVelocity(right_motor_velocity)
        print(f"Speed is right {left_motor_velocity} and left motor: {right_motor_velocity}")
        if distance < 0.1:
            self.__left_motor.setVelocity(0)
            self.__right_motor.setVelocity(0)
            print("Point goal!")
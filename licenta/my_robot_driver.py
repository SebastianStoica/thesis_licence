import rclpy
from geometry_msgs.msg import Twist,PointStamped
from math import atan2
import numpy as np
from sensor_msgs.msg import Range
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
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        self.__target_twist = Twist()
        self.__target_gps = PointStamped()
        self.goal_position = (1,0.5,200)
        self.max_linear_velocity = 9.0
        self.max_angular_velocity = 3.0
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.__node.create_subscription(PointStamped, 'gps_sensor', self.gps_sensor_callback, 10)

        self.__node.get_logger().info("Start thesis")
      
    def cmd_vel_callback(self, twist):
        self.__target_twist = twist
        
    def gps_sensor_callback(self,msg):
        self.__target_gps = msg


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        north = self.compass.getValues()
        angle = atan2(north[1], north[0])
        theta = np.degrees(angle)
        if theta <0:
            theta +=360

        x_pose = self.__target_gps.point.x
        y_pose = self.__target_gps.point.y
        print(f"Position Khepera is -> x: {x_pose}, y: {y_pose}, theta: {theta}")

        current_position = (x_pose, y_pose,theta)
        #print(current_position)
        delta_x = self.goal_position[0] - current_position[0]
        delta_y = self.goal_position[1] - current_position[1]
        print(delta_x,delta_y)
        distance = math.sqrt(delta_x**2 + delta_y**2)
        print(f"dif de punctul tinta este: {distance}")
     
        theta_difference = self.goal_position[2] - theta
        theta_error = np.degrees(atan2(math.sin(theta_difference),math.cos(theta_difference)))
        print(theta_error)
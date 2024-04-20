import rclpy
from geometry_msgs.msg import Twist,PointStamped
from math import atan2
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
    
        self.timeStep = 1
        self.__left_motor = self.robot.getDevice('left wheel motor')
        self.__right_motor = self.robot.getDevice('right wheel motor')
        try:
            self.compass = self.robot.getDevice('compass')
            self.compass.enable(1)
            
        except Exception as e: 
            print(e)
        self.max_motor_velocity = 19.1
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        self.__target_twist = Twist()
        self.__target_gps = PointStamped()
        self.lidar_data = LaserScan()
        self.max_linear_velocity = 19.0
        self.max_angular_velocity = 0.5
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.__node.create_subscription(PointStamped, 'gps_sensor', self.gps_sensor_callback, 10)
        self.__node.create_subscription(LaserScan, 'lidar_sensor', self.lidar_sensor_callback, 10)
        self.compass_publisher = self.__node.create_publisher(Float64, 'compass_sensor', 10)

        self.__node.get_logger().info("Start thesis")

    

    def cmd_vel_callback(self, twist):
        self.__target_twist = twist
        self.linear_velocity = self.__target_twist.linear.x
        self.angular_velocity = self.__target_twist.angular.z
    
    def gps_sensor_callback(self,msg):
        self.__target_gps = msg

    def lidar_sensor_callback(self,msg):
        self.lidar_data = msg

    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        compass_msg = Float64()
        north = self.compass.getValues()
        angle = -atan2(north[1], north[0])
        theta = np.degrees(angle) - 270
        if theta < 0:
            theta = theta + 360
        x_pose = self.__target_gps.point.x
        y_pose = self.__target_gps.point.y
        compass_msg.data = theta

        self.compass_publisher.publish(compass_msg)
        current_position = (x_pose, y_pose, theta)
       
        forward_speed = self.linear_velocity
        angular_speed = self.angular_velocity
        command_motor_left = (forward_speed - angular_speed * 0.8) / 0.5
        command_motor_right = (forward_speed + angular_speed * 0.8) / 0.5
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

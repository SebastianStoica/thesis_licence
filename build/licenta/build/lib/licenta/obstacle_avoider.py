import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MAX_RANGE = 0.15
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 2.0  


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) 
        #
        self.lidar_msg = LaserScan()
        self.create_subscription(LaserScan, 'lidar_sensor', self.lidar_sensor_callback, 10)
        r = self.lidar_msg.ranges

    def lidar_sensor_callback(self,msg):
        self.lidar_msg = msg
        

def main(args=None):
    rclpy.init(args=args)

    avoider = ObstacleAvoider()

    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

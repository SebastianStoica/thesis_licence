import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.lidar_data = LaserScan()
        self.create_subscription(LaserScan, 'lidar_sensor', self.lidar_sensor_callback, 10)
        self.create_timer(1, self.step_obstclate)


    def lidar_sensor_callback(self,msg):
        self.lidar_data = msg
    
    def step_obstclate(self):
        data = self.lidar_data.ranges
      

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PointStamped
import numpy as np
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R

class WebotsRobotTFBroadcaster(Node):
    def __init__(self):
        super().__init__('webots_robot_tf_broadcaster')

           
        self.theta = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscr = self.create_subscription(PointStamped, 'gps_sensor', self.gps_data, 10)
        self.subscriber = self.create_subscription(Float64, 'compass_sensor', self.compass_callback, 10)

        self.position_x = None
        self.position_y = None
        self.position_z = None
        self.orientation_z = None
        self.orientation_w = None
        self.create_timer(0.01, self.broadcast_pose)

    def compass_callback(self,msg):
        self.theta = np.radians(msg.data)
        

    def gps_data(self, msg):
        self.position_x = msg.point.x
        self.position_y = msg.point.y
    
    def broadcast_pose(self):
        
        if self.position_x is None or self.position_y is None:
            #self.get_logger().warn('No GPS data received yet, skipping broadcast')
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        
        matrix_rotation = R.from_matrix([[np.cos(self.theta), -np.sin(self.theta),0],
                                         [np.sin(self.theta),np.cos(self.theta),0],
                                         [0,0,1]
                                         ])
        t.transform.rotation.x = matrix_rotation.as_quat()[0]
        t.transform.rotation.y = matrix_rotation.as_quat()[1]
        t.transform.rotation.z = matrix_rotation.as_quat()[2]
        t.transform.rotation.w = matrix_rotation.as_quat()[3]
       

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    broadcaster = WebotsRobotTFBroadcaster()
   
    rclpy.spin(broadcaster)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

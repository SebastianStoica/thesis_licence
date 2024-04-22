import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
from shapely.geometry import Polygon, Point
import random
import numpy as np

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle')
        self.data = TFMessage()
        self.lidar_data = LaserScan()
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.create_subscription(LaserScan, '/lidar_sensor', self.lidar_sensor_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.obstacle_polygons = []
        self.start = (0, 0)
        self.goal = (2, 1)
        self.path = None
        self.obstacles = {
            'box0': {'pose': (1, 1), 'size': (0.1, 0.3),'rotation': (0,0,0.258819,0.965926)},
            'box1': {'pose': (0.299971, 1.25253), 'size': (0.1, 0.2), 'rotation':(0,0,-0.382682,0.92388)},
            'box2': {'pose': (1.42041, 0.263898), 'size': (0.1, 0.2), 'rotation': (0,0,0.707108,0.707105)},
            'box3': {'pose': (1.08685, 1.62249), 'size': (0.2, 0.2),'rotation': (0,0,-0.608761,0.793354)},
            'box4': {'pose': (1.9554, 1.6204), 'size': (0.1, 0.2),'rotation': (0,0,0.793355,0.60876)},
            'box5': {'pose': (0.6, 0.3), 'size': (0.1, 0.3), 'rotation':(0,0,0.923879,0.382686)}
        }

        self.create_timer(0.01,self.init_sensors)
        self.print_obstacles()

    def print_obstacles(self):
        for name, obstacle in self.obstacles.items():
            pose = obstacle['pose']
            size = obstacle['size']
            rotation = obstacle['rotation']
           
            self.get_logger().info(f"Obstacle {name}:")
            self.get_logger().info(f"    Pose: {pose}")
            self.get_logger().info(f"    Size: {size}")
           
            quaternion = R.from_quat(rotation)
            euler = quaternion.as_euler('zyx', degrees=True)
            angle_obstcl = euler[0]
            if angle_obstcl<0:
                angle_obstcl+=360
            self.get_logger().info(f"    Rotation (Euler): {angle_obstcl}")

           # self.get_logger().info(f"    vertices_A_X: {vi11x}, vertices_A_Y: {vi11y}")
            a= self.calc_init_obstcl(size)



            self.get_logger().info(f"    ini: {a}")
            self.get_logger().info(f"    iniAx: {a[0][0]}")
            self.get_logger().info(f"    ini: {len(a)}")
            b = list(self.calc_pose_obstcle(a,angle=angle_obstcl,pose=pose))
          #  self.get_logger().info(f" ABCD: : {b}")
            self.obstacle_polygon = Polygon(b)
            self.obstacle_polygons.append(self.obstacle_polygon)
           # self.get_logger().info(f" POLIGON: : {self.obstacle_polygons}")

    def calc_init_obstcl(self,size):
        A_INI = (0-size[0]/2,   0-size[1]/2)
        B_INI = (0+size[0]/2,   0-size[1]/2)
        C_INI = (0+size[0]/2,   0+size[1]/2)
        D_INI = (0-size[0]/2,   0+size[1]/2)
        return A_INI,B_INI,C_INI,D_INI
        
    def calc_pose_obstcle(self,a,angle,pose):
       
        A = (pose[0] + a[0][0]*np.cos(angle) - np.sin(angle)*a[0][1], pose[1] + a[0][0]*np.sin(angle) + np.cos(angle)*a[0][1])
        B = (pose[0] + a[1][0]*np.cos(angle) - np.sin(angle)*a[1][1], pose[1] + a[1][0]*np.sin(angle) + np.cos(angle)*a[1][1])
        C = (pose[0] + a[2][0]*np.cos(angle) - np.sin(angle)*a[2][1], pose[1] + a[2][0]*np.sin(angle) + np.cos(angle)*a[2][1])
        D = (pose[0] + a[3][0]*np.cos(angle) - np.sin(angle)*a[3][1], pose[1] + a[3][0]*np.sin(angle) + np.cos(angle)*a[3][1])
        return A,B,C,D
    
    def generate_rrt(self):
        start = self.start
        goal = self.goal
        step_size = 0.1 
        max_iter = 1000
        bounds = [(0, 2.3), (0, 2.3)]  

        rrt_planner = RRT(start, goal, step_size, max_iter, bounds,self.obstacle_polygons)
        path = rrt_planner.generate_rrt()

        return path


    def lidar_sensor_callback(self,msg):
        self.lidar_data = msg.ranges

    def init_sensors(self):
       # print(self.data.transforms)
        pass

    def tf_callback(self,msg):
        self.data = msg


    def print_path(self):
        if self.path:
            self.get_logger().info("Path generated by RRT:")
            for point in self.path:
                self.get_logger().info(f"Point: {point}")
        else:
            self.get_logger().info("No path generated.")


class RRT:
    def __init__(self, start, goal, step_size, max_iter, bounds, obstacle_polygons):
        self.start = start
        self.goal = goal
        self.step_size = step_size
        self.max_iter = max_iter
        self.bounds = bounds
        self.tree = {}
        self.obstacle_polygons = obstacle_polygons
   
    def generate_rrt(self):

        self.tree[self.start] = None
        for _ in range(self.max_iter):

            random_point = self.generate_random_point()
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.extend(nearest_point, random_point)

            if new_point and not self.is_colliding(new_point) :
                self.tree[new_point] = nearest_point
                if self.is_goal_reached(new_point):
                    return self.construct_path(new_point)
        return None

    def is_colliding(self, point):
        for obstacle_polygon in self.obstacle_polygons:
            if obstacle_polygon.contains(Point(point)):
                return True
        return False

    def generate_random_point(self):
        return (random.uniform(self.bounds[0][0], self.bounds[0][1]),
                random.uniform(self.bounds[1][0], self.bounds[1][1]))

    def find_nearest_point(self, point):
        min_distance = float('inf')
        nearest_point = None
        for p in self.tree:
            distance = np.linalg.norm(np.array(p) - np.array(point))
            if distance < min_distance:
                min_distance = distance
                nearest_point = p
        return nearest_point

    def extend(self, from_point, to_point):
        direction = np.array(to_point) - np.array(from_point)
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = (direction / distance) * self.step_size
            new_point = tuple(np.array(from_point) + direction)
            if not self.is_collision_free(from_point, new_point):
                return None  
            return new_point
        return to_point
    
    def is_collision_free(self, from_point, to_point):
        robot_radius = 0.065  
        for obstacle_polygon in self.obstacle_polygons:
            circle = Point(to_point).buffer(robot_radius)
            if circle.intersects(obstacle_polygon):
                return False
        return True


    def is_goal_reached(self, point):
        distance_to_goal = np.linalg.norm(np.array(point) - np.array(self.goal))
        return distance_to_goal < self.step_size

    def construct_path(self, end_point):
        path = [end_point]
        while self.tree[end_point]:
            end_point = self.tree[end_point]
            path.append(end_point)
        return list(reversed(path))
    
    
def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    path = avoider.generate_rrt()
    if path:
        avoider.get_logger().info("Path generated by RRT:")
        for point in path:
            avoider.get_logger().info(f"Point: {point}")
    else:
        avoider.get_logger().info("Failed to generate path by RRT.")
    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


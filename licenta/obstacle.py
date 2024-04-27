import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PointStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
from shapely.geometry import Polygon, Point,LineString
import random
import math
import numpy as np
from std_msgs.msg import Float64
import matplotlib.pyplot as plt


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
        self.path_points_x = []
        self.path_points_y =[]
        self.total_points = []
        self.theta = Float64()
        self.position  = PointStamped()
        self.create_subscription(LaserScan, '/lidar_sensor', self.lidar_sensor_callback, 10)
        self.subscriber = self.create_subscription(Float64, 'compass_sensor', self.compass_callback, 10)
        self.subscr = self.create_subscription(PointStamped, 'gps_sensor', self.gps_data, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.obstacle_polygons = []
        self.start = (0.0, 0.0)
        self.goal = (2.04364, 2.03687)
        self.obstacles = {
            'box0': {'pose': (1, 1), 'size': (0.1, 0.3),'rotation': (0.523599)},
            'box1': {'pose': (0.299971, 1.25253), 'size': (0.1, 0.2), 'rotation':(-0.785395)},
            'box2': {'pose': (1.42041, 0.263898), 'size': (0.1, 0.2), 'rotation': (1.5708)},
            'box3': {'pose': (1.08685, 1.62249), 'size': (0.2, 0.2),'rotation': (-1.309)},
            'box4': {'pose': (1.9554, 1.6204), 'size': (0.1, 0.2),'rotation': (1.8326)},
            'box5': {'pose': (0.6, 0.3), 'size': (0.1, 0.3), 'rotation':(2.35619)}
        }
        self.create_timer(0.001,self.move_khepera)

    def compass_callback(self,msg):
        self.theta = msg

    def gps_data(self,msg):
        self.position = msg

    def move_khepera(self):

        current_angle = math.radians(self.theta.data)
       # print(f"unghiul curent este: {current_angle}")
        points_current = self.position.point.x,self.position.point.y
        #print(points_current)
        
        twist = Twist()
        if len(self.path_points_x)>1 and len(self.path_points_y) > 1:
            x_next = self.path_points_x[1]
            y_next =self.path_points_y[1]

           # print(f"x next is : {x_next} and y next is: {y_next}")
            delta_x = x_next - points_current[0]
            delta_y = y_next - points_current[1]
            angle = math.atan2(delta_y,delta_x)

            angle_diff = angle - current_angle
            angle_diff = np.angle(np.exp(1j * angle_diff))
           
           # self.get_logger().info(f"angle_diff este : {angle_diff}")

            distance = math.sqrt(delta_x**2 + delta_y**2)

            if abs(angle_diff) > 0.01:
                twist.angular.z = np.sign(angle_diff)*0.06
                twist.linear.x = 0.0
            if distance > 0.01 and twist.angular.z ==0.0:
                twist.angular.z =0.0
                twist.linear.x = 0.1
            if distance <0.01 and twist.angular.z ==0.0:
                twist.linear.x =0.0
                twist.angular.z =0.0
 
            if twist.angular.z ==0.0 and twist.linear.x ==0.0:
                self.path_points_x.pop(0)
                self.path_points_y.pop(0)
                #print(f"noul punct este: {self.path_points_x} and {self.path_points_y}")
           # print(len(self.path_points_x))
          #  points_current = x_next,y_next
        else:
            dx = self.goal[0] - points_current[0]
            dy = self.goal[1] - points_current[1]
            ang = math.atan2(dy,dx)
            ag_dif = ang - current_angle
            ag_dif = np.angle(np.exp(1j * ag_dif))
            dis_to_goal = math.sqrt(dx**2 + dy**2)
            if abs(ag_dif) > 0.01:
                twist.angular.z = np.sign(ag_dif)*0.03
                twist.linear.x = 0.0
            if dis_to_goal > 0.01 and twist.angular.z ==0:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            if dis_to_goal <0.01 and twist.angular.z ==0.0:
                twist.linear.x =0.0
                twist.angular.z =0.0
                self.get_logger().info("PUNCT ATINS")
        self.publisher_.publish(twist)

    def print_obstacles(self):
        for name, obstacle in self.obstacles.items():
            pose = obstacle['pose']
            size = obstacle['size']
            rotation = obstacle['rotation']
            euler_angle = rotation
            a = self.calc_init_obstcl(size)
            b = list(self.calc_pose_obstcle(a, angle=euler_angle, pose=pose))
            self.obstacle_polygon = Polygon(b)
            self.obstacle_polygons.append(self.obstacle_polygon)
        #self.get_logger().info(f" POLIGON: : {self.obstacle_polygons}")


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
        max_iter = 10000
        bounds = [(0, 2.3), (0, 2.3)]  

        rrt_planner = RRT(start, goal, step_size, max_iter, bounds, self.obstacle_polygons)
        path = rrt_planner.generate_rrt()

        all_points = rrt_planner.get_all_points()
        for point in all_points:
            self.total_points.append(point)

        if path:
            print("Generated path:")
            for point in path:
                print(f"Point: {point}")
                if any(obstacle_polygon.contains(Point(point)) for obstacle_polygon in self.obstacle_polygons):
                    print("Point is inside an obstacle!")
        else:
            print("Failed to generate path!")
        self.get_logger().info(f"path ul esteee : {path}")
        self.path_points_x = [point[0] for point in path]
        self.path_points_y= [point[1] for point in path]
        self.get_logger().info(f"points y is {self.path_points_y} and points x is {self.path_points_x}")
        return path

        
    def lidar_sensor_callback(self,msg):
        self.lidar_data = msg.ranges



    def tf_callback(self,msg):
        self.data = msg


    def init_sensors(self):
        if self.path_points:
            self.get_logger().info("Path generated by RRT:")
            for point in self.path_points:
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
        self.obstacle_polygons = obstacle_polygons
        self.robot_radius = 0.08
        self.tree = {start: None}

    def generate_random_point(self):
        while True:
            random_point = (random.uniform(self.bounds[0][0], self.bounds[0][1]),
                            random.uniform(self.bounds[1][0], self.bounds[1][1]))
  
            if not any(obstacle_polygon.contains(Point(random_point)) for obstacle_polygon in self.obstacle_polygons):
                return random_point

    def is_collision_free(self, from_point, to_point):
        line = LineString([from_point, to_point])
        for obstacle_polygon in self.obstacle_polygons:
            if line.intersects(obstacle_polygon.buffer(self.robot_radius)):
                return False
        return True

    def find_nearest_point(self, point):
        min_distance = float('inf')
        nearest_point = None
        for p in self.tree:
            distance = np.linalg.norm(np.array(p) - np.array(point))
            if distance < min_distance:
                min_distance = distance
                nearest_point = p
        return nearest_point

    def get_all_points(self):
        return list(self.tree.keys())

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

    def is_goal_reached(self, point):
        distance_to_goal = np.linalg.norm(np.array(point) - np.array(self.goal))
        return distance_to_goal < self.step_size

    def construct_path(self, end_point):
        path = [end_point]
        while self.tree[end_point]:
            end_point = self.tree[end_point]
            path.append(end_point)
        return list(reversed(path))

    def generate_rrt(self):
        for _ in range(self.max_iter):
            random_point = self.generate_random_point()
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.extend(nearest_point, random_point)
            if new_point:
                self.tree[new_point] = nearest_point
                if self.is_goal_reached(new_point):
                    path = self.construct_path(new_point)
                    return path  
        return None
def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    avoider.print_obstacles()
    path = avoider.generate_rrt()
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    
    plt.figure()
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', label='Path')

    all_x_coords = [point[0] for point in avoider.total_points]
    all_y_coords = [point[1] for point in avoider.total_points]
    plt.scatter(all_x_coords, all_y_coords, color='grey', label='Generated Points')
    
    for name, obstacle in avoider.obstacles.items():
        pose = obstacle['pose']
        size = obstacle['size']
        rotation = obstacle['rotation']
        a = avoider.calc_init_obstcl(size)
        b = list(avoider.calc_pose_obstcle(a, angle=rotation, pose=pose))
        x_coords = [point[0] for point in b]
        y_coords = [point[1] for point in b]
        plt.fill(x_coords, y_coords, 'r', alpha=0.5)

    start_x, start_y = avoider.start
    plt.scatter(start_x, start_y, color='green', label='Start')
    
    goal_x, goal_y = avoider.goal
    plt.scatter(goal_x, goal_y, color='blue', label='Goal')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path with Obstacles')
    plt.grid(True)
    plt.legend()
    plt.show()

    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


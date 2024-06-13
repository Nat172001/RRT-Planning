#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
import random
from std_msgs.msg import String
from math import pow,sqrt,atan2,pi

# Definitions for grid boundaries and size
GRID_CORNERS = [(-5, -5), (-5, 5), (5, 5), (5, -5), (-5, -5)]
GRID_SIZE_X = 475
GRID_SIZE_Y = 475

def pose_to_grid(x, y):
    grid_x = int((x - GRID_CORNERS[0][0]) / (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) * (GRID_SIZE_X - 1))
    grid_y = int((y - GRID_CORNERS[0][1]) / (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) * (GRID_SIZE_Y - 1))
    return max(0, min(grid_x, GRID_SIZE_X - 1)), max(0, min(grid_y, GRID_SIZE_Y - 1))
    
def grid_to_pose(grid_x, grid_y):
    cell_width = (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) / (GRID_SIZE_X - 1)
    cell_height = (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) / (GRID_SIZE_Y - 1)
    x = GRID_CORNERS[0][0] + grid_x * cell_width
    y = GRID_CORNERS[0][1] + grid_y * cell_height
    return x, y#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
import random
from std_msgs.msg import String
from math import pow,sqrt,atan2,pi

# Definitions for grid boundaries and size
GRID_CORNERS = [(-5, -5), (-5, 5), (5, 5), (5, -5), (-5, -5)]
GRID_SIZE_X = 475
GRID_SIZE_Y = 475

def pose_to_grid(x, y):
    grid_x = int((x - GRID_CORNERS[0][0]) / (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) * (GRID_SIZE_X - 1))
    grid_y = int((y - GRID_CORNERS[0][1]) / (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) * (GRID_SIZE_Y - 1))
    return max(0, min(grid_x, GRID_SIZE_X - 1)), max(0, min(grid_y, GRID_SIZE_Y - 1))
    
def grid_to_pose(grid_x, grid_y):
    cell_width = (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) / (GRID_SIZE_X - 1)
    cell_height = (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) / (GRID_SIZE_Y - 1)
    x = GRID_CORNERS[0][0] + grid_x * cell_width
    y = GRID_CORNERS[0][1] + grid_y * cell_height
    return x, y

class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.children = []  # List to store children nodes
        self.parent = None  # Parent node

class RRT:
    def __init__(self, start, goal, grid,robot_radius,step_size):
        self.tree = TreeNode(start[0], start[1])  # Root node 
        self.goal = TreeNode(goal[0], goal[1])    # Goal node
        self.grid = grid    
        self.robot_radius = robot_radius
        self.step_size = step_size  
        self.goal_region = robot_radius        # Radius around the goal
        self.nodes = [self.tree]    # store all nodes 

    def add_child(self, node, parent):
        parent.children.append(node)
        node.parent = parent
        self.nodes.append(node)

    def sampling(self):
        x = random.randint(0, self.grid.shape[0] - 1)
        y = random.randint(0, self.grid.shape[1] - 1)
        return np.array([x, y])

    def steer(self, start, end):
        # Steer from the start point towards the end point by the step size
        direction = np.array(end) - np.array(start)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            new_node = end
        else:
            unit_vector = direction / distance
            new_node = (np.array(start) + unit_vector * self.step_size).astype(int)

        # Check for obstacles along the path from start to new_node
        path_cells = self.bresenham(start[0],start[1], new_node[0],new_node[1])
        for cell in path_cells:
            if not self.is_collision_free(TreeNode(cell[0], cell[1])):
                return None, True  # Path is not collision-free
        return new_node, False

    def bresenham(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        x, y = x1, y1

        line_points = []
        while True:
            line_points.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return line_points

    def is_collision_free(self, node):
        x, y = int(node.x), int(node.y)
        if x < self.robot_radius or y < self.robot_radius or x >= self.grid.shape[1] - self.robot_radius or y >= self.grid.shape[0] - self.robot_radius:
            return False
        # Check if any cell within the circular robot's radius is occupied
        for i in range(-self.robot_radius, self.robot_radius + 1):
            for j in range(-self.robot_radius, self.robot_radius + 1):
                if self.grid[y + i, x + j] != 0:
                    return False
        return True

    def nearest_node(self, point):
        closest_node = None
        min_distance = float('inf')
        for node in self.nodes:
            distance = np.linalg.norm(np.array([node.x, node.y]) - np.array(point))
            if distance < min_distance:
                min_distance = distance
                closest_node = node
        return closest_node

    def within_goal_region(self, node):
        if np.linalg.norm(np.array([node.x, node.y]) - np.array([self.goal.x, self.goal.y])) <= self.goal_region:
            return True
        return False

    def build_tree(self, max_iterations=1000):
        plt.ion()  # interactive mode
        fig, ax = plt.subplots() 
        ax.imshow(self.grid, cmap='binary')  
        ax.plot(self.tree.x, self.tree.y, 'ro', label='Start') 
        ax.plot(self.goal.x, self.goal.y, 'bo', label='Goal')    
        plt.xlim(0, self.grid.shape[0])  
        plt.ylim(0, self.grid.shape[1])  
        plt.draw()  # Draw the initial plot

        for i in range(max_iterations):
            random_point = self.sampling()  
            nearest = self.nearest_node(random_point)  
            new_point, collision = self.steer([nearest.x, nearest.y], random_point) 
            if collision == True: continue
            new_node = TreeNode(new_point[0], new_point[1]) 
            self.add_child(new_node, nearest) 
            ax.plot([nearest.x, new_node.x], [nearest.y, new_node.y], 'y-')  
            circle = plt.Circle((new_node.x, new_node.y), self.robot_radius, color='g', alpha=0.3)
            ax.add_patch(circle)
            plt.draw()  
            plt.pause(0.005)  

            if self.within_goal_region(new_node):  
                print("Goal reached successfully in simulation!",flush=True)
                plt.ioff()  
                return new_node  

        plt.ioff()  
        return None  

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.counter =1
        self.initial_pose = None
        self.goal_pose = None
        self.og_publisher = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        self.path_pub = self.create_publisher(Path,"/path",10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Pose2D,"/gp",self.set_pose_variable,10)
        self.stop_pub = self.create_publisher(String, "/environment", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/points", 10)  # Increase publishing frequency if necessary
        self.robot_radius = 24  # 1 pixel = 0.02066116
        self.step_size = 25
        self.rrt = None
        self.path_markers = []  # Store path markers

        # Prompt user for environment selection
        self.env_selection = input("Select environment (1-5): ")
        if self.env_selection.isdigit():
            env_num = int(self.env_selection)
            if 1 <= env_num <= 5:
                self.grid = np.load(f'/home/nataraj/ros2_ws/src/planning/planning/env{env_num}.npy')
                self.publish_map()  # Publish the occupancy grid map

    def set_pose_variable(self, msg: Pose2D):
        print(f"received initial pose from to goal: {msg.x} {msg.y}",flush=True)
        self.initial_pose=[msg.x,msg._y]
        goal=PoseStamped()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        self.goal_pose_callback(goal)

    def initial_pose_callback(self, msg):
        self.initial_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.publish_marker(self.initial_pose, Marker.CYLINDER)
        self.flag = True

    def goal_pose_callback(self, msg):
        self.goal_pose = [msg.pose.position.x, msg.pose.position.y]
        self.publish_marker(self.goal_pose, Marker.SPHERE)

        if self.flag==False and self.counter==1:
            stop_msg = String()
            stop_msg.data = "stop"
            self.stop_pub.publish(stop_msg)
            self.counter = 0

        if self.initial_pose == None:
            return
                   
        if self.initial_pose is not None and self.goal_pose is not None:
            # Convert initial and goal poses to grid coordinates
            start = pose_to_grid(self.initial_pose[0], self.initial_pose[1])
            goal = pose_to_grid(self.goal_pose[0], self.goal_pose[1])

            self.rrt = RRT(start, goal, self.grid, self.robot_radius, self.step_size)
            goal_node = self.rrt.build_tree(max_iterations=1000)

            if goal_node:
                path = []
                current = goal_node
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(self.rrt.tree)
                path = path[::-1]
                path_points = []
                for p in path:
                    x, y = grid_to_pose(p.x, p.y)
                    path_points.append([x, y])

                # Publish path as a line strip marker
                self.publish_path_marker(path_points)
                self.publish_path(path_points)

            else:
                print("Failed to reach the goal",flush=True)

            self.initial_pose = None
            self.goal_pose = None
            self.counter=1
        self.flag =False

    def publish_map(self):
        # Publish the occupancy grid map
        og_publish = OccupancyGrid()
        og_publish.header.stamp = self.get_clock().now().to_msg()
        og_publish.header.frame_id = "world"
        og_publish.info.resolution = 0.02066116
        og_publish.info.width = GRID_SIZE_X
        og_publish.info.height = GRID_SIZE_Y
        og_publish.info.origin.position.x = -5.0
        og_publish.info.origin.position.y = -5.0
        og_publish.info.origin.position.z = 0.0
        og_publish.info.origin.orientation.x = 0.0
        og_publish.info.origin.orientation.y = 0.0
        og_publish.info.origin.orientation.z = 0.0
        og_publish.info.origin.orientation.w = 1.0

        og_array = []
        for i in range(GRID_SIZE_X):
            for j in range(GRID_SIZE_Y):
                if i < 4 or i > 471 or j < 4 or j > 471:
                    self.grid[i][j] = 1
                og_array.append(int(self.grid[i][j]) * 100)

        og_publish.data = og_array
        self.og_publisher.publish(og_publish)

    def publish_marker(self, pose, marker_type):
        marker_array = MarkerArray()  # Create a new MarkerArray for each marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker_type
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        if marker_type == 0:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif marker_type == 1:
            marker.color.g = 1.0
            marker.color.r = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)  # Add the marker to the MarkerArray
        self.marker_publisher.publish(marker_array)  # Publish the MarkerArray

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "world"  # Set the frame ID
        path_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

        for point in path_points:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        
    def publish_path_marker(self, path_points):
        marker_array = MarkerArray()  # Create a new MarkerArray for the path markers
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1  # Width of the line
        marker.color.b = 1.0
        marker.color.a = 1.0

        for point in path_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            marker.points.append(p)

        marker_array.markers.append(marker)  # Add the marker to the MarkerArray
        self.marker_publisher.publish(marker_array)  # Publish the MarkerArray
        

def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPlanner()
    rclpy.spin(rrt_planner)
    rrt_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.children = []  # List to store children nodes
        self.parent = None  # Parent node

class RRT:
    def __init__(self, start, goal, grid,robot_radius,step_size):
        self.tree = TreeNode(start[0], start[1])  # Root node 
        self.goal = TreeNode(goal[0], goal[1])    # Goal node
        self.grid = grid    
        self.robot_radius = robot_radius
        self.step_size = step_size  
        self.goal_region = robot_radius        # Radius around the goal
        self.nodes = [self.tree]    # store all nodes 

    def add_child(self, node, parent):
        parent.children.append(node)
        node.parent = parent
        self.nodes.append(node)

    def sampling(self):
        x = random.randint(0, self.grid.shape[0] - 1)
        y = random.randint(0, self.grid.shape[1] - 1)
        return np.array([x, y])

    def steer(self, start, end):
        # Steer from the start point towards the end point by the step size
        direction = np.array(end) - np.array(start)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            new_node = end
        else:
            unit_vector = direction / distance
            new_node = (np.array(start) + unit_vector * self.step_size).astype(int)

        # Check for obstacles along the path from start to new_node
        path_cells = self.bresenham(start[0],start[1], new_node[0],new_node[1])
        for cell in path_cells:
            if not self.is_collision_free(TreeNode(cell[0], cell[1])):
                return None, True  # Path is not collision-free
        return new_node, False

    def bresenham(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        x, y = x1, y1

        line_points = []
        while True:
            line_points.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return line_points

    def is_collision_free(self, node):
        x, y = int(node.x), int(node.y)
        if x < self.robot_radius or y < self.robot_radius or x >= self.grid.shape[1] - self.robot_radius or y >= self.grid.shape[0] - self.robot_radius:
            return False
        # Check if any cell within the circular robot's radius is occupied
        for i in range(-self.robot_radius, self.robot_radius + 1):
            for j in range(-self.robot_radius, self.robot_radius + 1):
                if self.grid[y + i, x + j] != 0:
                    return False
        return True

    def nearest_node(self, point):
        closest_node = None
        min_distance = float('inf')
        for node in self.nodes:
            distance = np.linalg.norm(np.array([node.x, node.y]) - np.array(point))
            if distance < min_distance:
                min_distance = distance
                closest_node = node
        return closest_node

    def within_goal_region(self, node):
        if np.linalg.norm(np.array([node.x, node.y]) - np.array([self.goal.x, self.goal.y])) <= self.goal_region:
            return True
        return False

    def build_tree(self, max_iterations=1000):
        plt.ion()  # interactive mode
        fig, ax = plt.subplots() 
        ax.imshow(self.grid, cmap='binary')  
        ax.plot(self.tree.x, self.tree.y, 'ro', label='Start') 
        ax.plot(self.goal.x, self.goal.y, 'bo', label='Goal')    
        plt.xlim(0, self.grid.shape[0])  
        plt.ylim(0, self.grid.shape[1])  
        plt.draw()  # Draw the initial plot

        for i in range(max_iterations):
            random_point = self.sampling()  
            nearest = self.nearest_node(random_point)  
            new_point, collision = self.steer([nearest.x, nearest.y], random_point) 
            if collision == True: continue
            new_node = TreeNode(new_point[0], new_point[1]) 
            self.add_child(new_node, nearest) 
            ax.plot([nearest.x, new_node.x], [nearest.y, new_node.y], 'y-')  
            circle = plt.Circle((new_node.x, new_node.y), self.robot_radius, color='g', alpha=0.3)
            ax.add_patch(circle)
            plt.draw()  
            plt.pause(0.001)  

            if self.within_goal_region(new_node):  
                print("Goal reached successfully in simulation!",flush=True)
                plt.ioff()  
                return new_node  

        plt.ioff()  
        return None  

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.counter =1
        self.initial_pose = None
        self.goal_pose = None
        self.og_publisher = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        self.path_pub = self.create_publisher(Path,"/path",10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Pose2D,"/gp",self.set_pose_variable,10)
        self.stop_pub = self.create_publisher(String, "/environment", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/points", 10)  # Increase publishing frequency if necessary
        self.robot_radius = 24  # 1 pixel = 0.02066116
        self.step_size = 25
        self.rrt = None
        self.path_markers = []  # Store path markers

        # Prompt user for environment selection
        self.env_selection = input("Select environment (1-5): ")
        if self.env_selection.isdigit():
            env_num = int(self.env_selection)
            if 1 <= env_num <= 5:
                self.grid = np.load(f'/home/nataraj/ros2_ws/src/planning/planning/env{env_num}.npy')
                self.publish_map()  # Publish the occupancy grid map

    def set_pose_variable(self, msg: Pose2D):
        print(f"received initial pose from to goal: {msg.x} {msg.y}",flush=True)
        self.initial_pose=[msg.x,msg._y]
        goal=PoseStamped()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        self.goal_pose_callback(goal)

    def initial_pose_callback(self, msg):
        self.initial_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.publish_marker(self.initial_pose, Marker.CYLINDER)
        self.flag = True

    def goal_pose_callback(self, msg):
        self.goal_pose = [msg.pose.position.x, msg.pose.position.y]
        self.publish_marker(self.goal_pose, Marker.SPHERE)

        if self.flag==False and self.counter==1:
            stop_msg = String()
            stop_msg.data = "stop"
            self.stop_pub.publish(stop_msg)
            self.counter = 0

        if self.initial_pose == None:
            return
                   
        if self.initial_pose is not None and self.goal_pose is not None:
            # Convert initial and goal poses to grid coordinates
            start = pose_to_grid(self.initial_pose[0], self.initial_pose[1])
            goal = pose_to_grid(self.goal_pose[0], self.goal_pose[1])

            self.rrt = RRT(start, goal, self.grid, self.robot_radius, self.step_size)
            goal_node = self.rrt.build_tree(max_iterations=1000)

            if goal_node:
                path = []
                current = goal_node
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(self.rrt.tree)
                path = path[::-1]
                path_points = []
                for p in path:
                    x, y = grid_to_pose(p.x, p.y)
                    path_points.append([x, y])

                # Publish path as a line strip marker
                self.publish_path_marker(path_points)
                self.publish_path(path_points)

            else:
                print("Failed to reach the goal",flush=True)

            self.initial_pose = None
            self.goal_pose = None
            self.counter=1
        self.flag =False

    def publish_map(self):
        # Publish the occupancy grid map
        og_publish = OccupancyGrid()
        og_publish.header.stamp = self.get_clock().now().to_msg()
        og_publish.header.frame_id = "world"
        og_publish.info.resolution = 0.02066116
        og_publish.info.width = GRID_SIZE_X
        og_publish.info.height = GRID_SIZE_Y
        og_publish.info.origin.position.x = -5.0
        og_publish.info.origin.position.y = -5.0
        og_publish.info.origin.position.z = 0.0
        og_publish.info.origin.orientation.x = 0.0
        og_publish.info.origin.orientation.y = 0.0
        og_publish.info.origin.orientation.z = 0.0
        og_publish.info.origin.orientation.w = 1.0

        og_array = []
        for i in range(GRID_SIZE_X):
            for j in range(GRID_SIZE_Y):
                if i < 4 or i > 471 or j < 4 or j > 471:
                    self.grid[i][j] = 1
                og_array.append(int(self.grid[i][j]) * 100)

        og_publish.data = og_array
        self.og_publisher.publish(og_publish)

    def publish_marker(self, pose, marker_type):
        marker_array = MarkerArray()  # Create a new MarkerArray for each marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker_type
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        if marker_type == 0:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif marker_type == 1:
            marker.color.g = 1.0
            marker.color.r = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)  # Add the marker to the MarkerArray
        self.marker_publisher.publish(marker_array)  # Publish the MarkerArray

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "world"  # Set the frame ID
        path_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

        for point in path_points:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        
    def publish_path_marker(self, path_points):
        marker_array = MarkerArray()  # Create a new MarkerArray for the path markers
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1  # Width of the line
        marker.color.b = 1.0
        marker.color.a = 1.0

        for point in path_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            marker.points.append(p)

        marker_array.markers.append(marker)  # Add the marker to the MarkerArray
        self.marker_publisher.publish(marker_array)  # Publish the MarkerArray
        

def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPlanner()
    rclpy.spin(rrt_planner)
    rrt_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


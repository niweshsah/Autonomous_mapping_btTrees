import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.map_data = None
        self.resolution = None
        self.origin = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.find_frontiers()

    def find_frontiers(self):
        """Detect frontier points"""
        if self.map_data is None:
            return

        frontier_points = []
        height, width = self.map_data.shape
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                if self.map_data[y, x] == -1:  # Unknown cell
                    neighbors = self.map_data[y-1:y+2, x-1:x+2]
                    if np.any(neighbors == 0):  # Check if adjacent to known free space
                        frontier_points.append((x, y))

        if frontier_points:
            goal = self.select_frontier(frontier_points)
            self.navigate_to_goal(goal)

    def select_frontier(self, frontiers):
        """Select the best frontier (nearest)"""
        robot_x, robot_y = 0, 0  # Get robot position (in real implementation use TF)
        frontiers.sort(key=lambda f: np.linalg.norm(np.array(f) - np.array([robot_x, robot_y])))
        best_frontier = frontiers[0]
        return best_frontier

    def navigate_to_goal(self, goal):
        """Send navigation goal to Nav2"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal[0] * self.resolution + self.origin.position.x
        goal_pose.pose.position.y = goal[1] * self.resolution + self.origin.position.y
        goal_pose.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(nav_goal)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.exceptions import ROSInterruptException
# from tf2_ros import TransformException, Buffer, TransformListener

# import py_trees
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Twist
# from nav_msgs.msg import OccupancyGrid
# from nav2_msgs.action import NavigateToPose

# class MapProcessor(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("MapProcessor")
#         self.node = node
    
#     def update(self):
#         return py_trees.common.Status.SUCCESS if self.node.map_data is not None else py_trees.common.Status.RUNNING

# class GetRobotPosition(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("GetRobotPosition")
#         self.node = node
    
#     def update(self):
#         position = self.node.get_robot_position()
#         if position is None:
#             return py_trees.common.Status.FAILURE
#         self.node.robot_position = position
#         return py_trees.common.Status.SUCCESS

# class CheckIfStuck(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("CheckIfStuck")
#         self.node = node
    
#     def update(self):
#         return py_trees.common.Status.FAILURE if self.node.is_stuck(self.node.robot_position) else py_trees.common.Status.SUCCESS

# class UnstuckRobot(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("UnstuckRobot")
#         self.node = node
    
#     def update(self):
#         self.node.unstuck_robot()
#         return py_trees.common.Status.SUCCESS

# class FindFrontiers(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("FindFrontiers")
#         self.node = node
    
#     def update(self):
#         self.node.frontiers = self.node.find_frontiers()
#         return py_trees.common.Status.SUCCESS if self.node.frontiers else py_trees.common.Status.FAILURE

# class SelectFrontier(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("SelectFrontier")
#         self.node = node
    
#     def update(self):
#         self.node.target_frontier = self.node.select_closest_frontier(self.node.frontiers, self.node.robot_position)
#         return py_trees.common.Status.SUCCESS if self.node.target_frontier else py_trees.common.Status.FAILURE

# class NavigateToFrontier(py_trees.behaviour.Behaviour):
#     def __init__(self, node):
#         super().__init__("NavigateToFrontier")
#         self.node = node
    
#     def update(self):
#         self.node.navigate_to_frontier(self.node.target_frontier)
#         return py_trees.common.Status.SUCCESS

# class FrontierExplorer(Node):
#     def __init__(self):
#         super().__init__('frontier_explorer')
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
#         self.map_data, self.map_info = None, None
#         self.last_position, self.stuck_count = None, 0
#         self.stuck_threshold = 10
#         self.robot_position, self.frontiers, self.target_frontier = None, [], None
#         self.tree = self.create_behavior_tree()
#         self.timer = self.create_timer(1.0, self.run_tree)

#     def create_behavior_tree(self):
#         root = py_trees.composites.Sequence(name="ExplorerRoot", memory=True)
#         unstuck_seq = py_trees.composites.Selector(name="Unstuck", memory=True)
#         unstuck_seq.add_children([CheckIfStuck(self), UnstuckRobot(self)])
#         explore_seq = py_trees.composites.Sequence(name="Explore", memory=True)
#         explore_seq.add_children([FindFrontiers(self), SelectFrontier(self), NavigateToFrontier(self)])
#         root.add_children([MapProcessor(self), GetRobotPosition(self), unstuck_seq, explore_seq])
#         return py_trees.trees.BehaviourTree(root)
    
#     def run_tree(self):
#         self.tree.tick()
    
#     def map_callback(self, msg):
#         self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
#         self.map_info = msg.info

#     def get_robot_position(self):
#         if not self.map_info:
#             return None
#         try:
#             transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             x, y = transform.transform.translation.x, transform.transform.translation.y
#             origin, res = self.map_info.origin.position, self.map_info.resolution
#             return int((x - origin.x) / res), int((y - origin.y) / res)
#         except TransformException:
#             return None

#     def is_stuck(self, pos):
#         if self.last_position and pos:
#             if (self.last_position[0] - pos[0])**2 + (self.last_position[1] - pos[1])**2 < 1:
#                 self.stuck_count += 1
#                 return self.stuck_count >= self.stuck_threshold
#         self.last_position = pos
#         return False

#     def unstuck_robot(self):
#         twist = Twist()
#         twist.linear.x, twist.angular.z = -0.1, 0.5
#         self.cmd_vel_pub.publish(twist)
#         self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
#         twist.linear.x, twist.angular.z = 0.0, 0.0
#         self.cmd_vel_pub.publish(twist)

#     def find_frontiers(self):
#         return [(x, y) for y in range(1, self.map_data.shape[0]-1) for x in range(1, self.map_data.shape[1]-1)
#                 if self.map_data[y, x] == -1 and any(self.map_data[y+dy, x+dx] == 0 for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)])]

#     def select_closest_frontier(self, frontiers, pos):
#         return min(frontiers, key=lambda f: (f[0]-pos[0])**2 + (f[1]-pos[1])**2, default=None)

#     def navigate_to_frontier(self, frontier):
#         if not frontier or not self.map_info:
#             return
#         goal = PoseStamped()
#         goal.header.frame_id, goal.header.stamp = "map", self.get_clock().now().to_msg()
#         origin, res = self.map_info.origin.position, self.map_info.resolution
#         goal.pose.position.x, goal.pose.position.y = frontier[0] * res + origin.x, frontier[1] * res + origin.y
#         goal.pose.orientation.w = 1.0
#         self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))

# def main():
#     rclpy.init()
#     explorer = FrontierExplorer()
#     rclpy.spin(explorer)
#     explorer.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

















import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException
from tf2_ros import TransformException, Buffer, TransformListener

import py_trees
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

class MapProcessor(py_trees.behaviour.Behaviour):
    """Check if map data is available."""
    def __init__(self, node):
        super().__init__("MapProcessor")
        self.node = node
    
    def update(self):
        if self.node.map_data is not None:
            self.node.get_logger().info("Map data received.")
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().warn("Waiting for map data...")
        return py_trees.common.Status.RUNNING

class GetRobotPosition(py_trees.behaviour.Behaviour):
    """Retrieve the robot's current position."""
    def __init__(self, node):
        super().__init__("GetRobotPosition")
        self.node = node
    
    def update(self):
        position = self.node.get_robot_position()
        if position is None:
            self.node.get_logger().error("Failed to get robot position.")
            return py_trees.common.Status.FAILURE
        self.node.robot_position = position
        self.node.get_logger().info(f"Robot position: {position}")
        return py_trees.common.Status.SUCCESS

class CheckIfStuck(py_trees.behaviour.Behaviour):
    """Check if the robot is stuck."""
    def __init__(self, node):
        super().__init__("CheckIfStuck")
        self.node = node
    
    def update(self):
        if self.node.is_stuck(self.node.robot_position):
            self.node.get_logger().warn("Robot is stuck.")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

class UnstuckRobot(py_trees.behaviour.Behaviour):
    """Try to unstuck the robot."""
    def __init__(self, node):
        super().__init__("UnstuckRobot")
        self.node = node
    
    def update(self):
        self.node.unstuck_robot()
        self.node.get_logger().info("Attempting to unstuck robot.")
        return py_trees.common.Status.SUCCESS

class FindFrontiers(py_trees.behaviour.Behaviour):
    """Identify unexplored frontiers."""
    def __init__(self, node):
        super().__init__("FindFrontiers")
        self.node = node
    
    def update(self):
        self.node.frontiers = self.node.find_frontiers()
        if self.node.frontiers:
            self.node.get_logger().info(f"Found {len(self.node.frontiers)} frontiers.")
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().warn("No frontiers found.")
        return py_trees.common.Status.FAILURE

class SelectFrontier(py_trees.behaviour.Behaviour):
    """Select the closest frontier for navigation."""
    def __init__(self, node):
        super().__init__("SelectFrontier")
        self.node = node
    
    def update(self):
        self.node.target_frontier = self.node.select_closest_frontier(self.node.frontiers, self.node.robot_position)
        if self.node.target_frontier:
            self.node.get_logger().info(f"Selected frontier: {self.node.target_frontier}")
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().error("Failed to select a frontier.")
        return py_trees.common.Status.FAILURE

class NavigateToFrontier(py_trees.behaviour.Behaviour):
    """Navigate towards the selected frontier."""
    def __init__(self, node):
        super().__init__("NavigateToFrontier")
        self.node = node
    
    def update(self):
        self.node.navigate_to_frontier(self.node.target_frontier)
        self.node.get_logger().info("Navigating to frontier.")
        return py_trees.common.Status.SUCCESS

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.map_data, self.map_info = None, None
        self.last_position, self.stuck_count = None, 0
        self.stuck_threshold = 10
        self.robot_position, self.frontiers, self.target_frontier = None, [], None
        self.tree = self.create_behavior_tree()
        self.timer = self.create_timer(1.0, self.run_tree)

    def create_behavior_tree(self):
        root = py_trees.composites.Sequence(name="ExplorerRoot", memory=True)
        unstuck_seq = py_trees.composites.Selector(name="Unstuck", memory=True)
        unstuck_seq.add_children([CheckIfStuck(self), UnstuckRobot(self)])
        explore_seq = py_trees.composites.Sequence(name="Explore", memory=True)
        explore_seq.add_children([FindFrontiers(self), SelectFrontier(self), NavigateToFrontier(self)])
        root.add_children([MapProcessor(self), GetRobotPosition(self), unstuck_seq, explore_seq])
        return py_trees.trees.BehaviourTree(root)
    
    def run_tree(self):
        self.tree.tick()
    
    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return int((transform.transform.translation.x - self.map_info.origin.position.x) / self.map_info.resolution), \
                   int((transform.transform.translation.y - self.map_info.origin.position.y) / self.map_info.resolution)
        except TransformException as e:
            self.get_logger().error(f"TransformException: {e}")
            return None

    def unstuck_robot(self):
        twist = Twist()
        twist.linear.x, twist.angular.z = -0.1, 0.5
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Unstuck action performed.")

    def navigate_to_frontier(self, frontier):
        if not frontier:
            return
        goal = PoseStamped()
        goal.header.frame_id, goal.header.stamp = "map", self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = frontier[0] * self.map_info.resolution + self.map_info.origin.position.x, \
                                                      frontier[1] * self.map_info.resolution + self.map_info.origin.position.y
        goal.pose.orientation.w = 1.0
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))

if __name__ == '__main__':
    rclpy.init()
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

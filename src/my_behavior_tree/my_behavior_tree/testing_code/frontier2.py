#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

# import numpy as np
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import PoseStamped, Point
# from nav2_msgs.action import NavigateToPose

# class SimpleFrontierExplorer(Node):
#     def __init__(self):
#         super().__init__('simple_frontier_explorer')
        
#         # TF2 setup for robot position
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # Map data storage
#         self.map_data = None
#         self.map_info = None
        
#         # Navigation client
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
#         # Map subscriber
#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             'map',
#             self.map_callback,
#             10)
            
#         # Exploration timer (1Hz)
#         self.timer = self.create_timer(1.0, self.explore)

#     def map_callback(self, msg):
#         """Store the latest map data"""
#         self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
#         self.map_info = msg.info
#         self.get_logger().info("Received new map")

#     def get_robot_position(self):
#         """Get current robot position in map coordinates"""
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 'map',
#                 'base_link',
#                 rclpy.time.Time())
            
#             # Convert to map grid coordinates
#             x = transform.transform.translation.x
#             y = transform.transform.translation.y
#             origin = self.map_info.origin.position
#             resolution = self.map_info.resolution
            
#             map_x = int((x - origin.x) / resolution)
#             map_y = int((y - origin.y) / resolution)
            
#             return (map_x, map_y)
            
#         except TransformException as ex:
#             self.get_logger().warn(f'TF error: {ex}')
#             return None

#     def find_frontiers(self):
#         """Simple frontier detection"""
#         if self.map_data is None:
#             return []
            
#         frontiers = []
#         height, width = self.map_data.shape
        
#         for y in range(1, height-1):
#             for x in range(1, width-1):
#                 if self.map_data[y, x] == -1:  # Unknown cell
#                     # Check adjacent cells for free space
#                     if (self.map_data[y-1, x] == 0 or self.map_data[y+1, x] == 0 or
#                         self.map_data[y, x-1] == 0 or self.map_data[y, x+1] == 0):
#                         frontiers.append((x, y))
        
#         return frontiers

#     def select_closest_frontier(self, frontiers, robot_pos):
#         """Select the frontier closest to the robot"""
#         if not frontiers or not robot_pos:
#             return None
            
#         # Simple distance-based selection
#         rx, ry = robot_pos
#         closest = min(frontiers, key=lambda f: (f[0]-rx)**2 + (f[1]-ry)**2)
#         return closest

#     def navigate_to_frontier(self, frontier):
#         """Send navigation goal to the frontier"""
#         if not frontier:
#             return
            
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = "map"
#         goal_pose.header.stamp = self.get_clock().now().to_msg()
        
#         # Convert map coordinates to world coordinates
#         origin = self.map_info.origin.position
#         resolution = self.map_info.resolution
        
#         goal_pose.pose.position.x = frontier[0] * resolution + origin.x
#         goal_pose.pose.position.y = frontier[1] * resolution + origin.y
#         goal_pose.pose.orientation.w = 1.0  # Default orientation
        
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = goal_pose
        
#         self.nav_client.wait_for_server()
#         self.nav_client.send_goal_async(goal_msg)
#         self.get_logger().info(f"Navigating to frontier at ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")

#     def explore(self):
#         """Main exploration routine"""
#         if self.map_data is None:
#             self.get_logger().warn("Waiting for map data...")
#             return
            
#         robot_pos = self.get_robot_position()
#         if not robot_pos:
#             self.get_logger().warn("Cannot get robot position")
#             return
            
#         frontiers = self.find_frontiers()
#         if not frontiers:
#             self.get_logger().info("No frontiers found - exploration complete?")
#             return
            
#         target = self.select_closest_frontier(frontiers, robot_pos)
#         self.navigate_to_frontier(target)

# def main(args=None):
#     rclpy.init(args=args)
#     explorer = SimpleFrontierExplorer()
    
#     try:
#         rclpy.spin(explorer)
#     except KeyboardInterrupt:

#         explorer.get_logger().info("Exploration stopped")
#     finally:
#         explorer.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose


class SimpleFrontierExplorer(Node):
    def __init__(self):
        super().__init__('simple_frontier_explorer')
        
        # TF2 setup for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Map data storage
        self.map_data = None
        self.map_info = None
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        # Velocity publisher for unstuck movement
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
            
        # Exploration timer (1Hz)
        self.timer = self.create_timer(1.0, self.explore)

        # Stuck detection variables
        self.last_position = None
        self.stuck_count = 0
        self.stuck_threshold = 10  # Number of cycles before considering the robot stuck

    def map_callback(self, msg):
        """Store the latest map data"""
        try:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.get_logger().info("Received new map")
        except Exception as e:
            self.get_logger().error(f"Error processing map data: {str(e)}")

    def get_robot_position(self):
        """Get current robot position in map coordinates"""
        if self.map_info is None:
            self.get_logger().warn("No map info available yet.")
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            
            # Convert to map grid coordinates
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            origin = self.map_info.origin.position
            resolution = self.map_info.resolution
            
            map_x = int((x - origin.x) / resolution)
            map_y = int((y - origin.y) / resolution)
            
            return (map_x, map_y)
            
        except TransformException as ex:
            self.get_logger().warn(f'TF error: {ex}')
            return None

    def is_stuck(self, current_pos):
        """Check if the robot is stuck at the same position"""
        if self.last_position and current_pos:
            distance_moved = (self.last_position[0] - current_pos[0])**2 + (self.last_position[1] - current_pos[1])**2
            
            if distance_moved < 1:  # Robot has barely moved
                self.stuck_count += 1
                if self.stuck_count >= self.stuck_threshold:
                    return True
            else:
                self.stuck_count = 0  # Reset if the robot moves
        self.last_position = current_pos
        return False

    def unstuck_robot(self):
        """Try to move the robot slightly to get it unstuck"""
        self.get_logger().warn("Trying to unstuck the robot...")
        
        twist = Twist()
        
        # Move backward slightly
        twist.linear.x = -0.1  # Small backward movement
        self.cmd_vel_pub.publish(twist)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # Rotate a little
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Small rotation
        self.cmd_vel_pub.publish(twist)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # Stop movement
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info("Unstuck movement executed.")

    def find_frontiers(self):
        """Simple frontier detection"""
        if self.map_data is None:
            self.get_logger().warn("No map data available.")
            return []
            
        frontiers = []
        height, width = self.map_data.shape
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                if self.map_data[y, x] == -1:  # Unknown cell
                    # Check adjacent cells for free space
                    if (self.map_data[y-1, x] == 0 or self.map_data[y+1, x] == 0 or
                        self.map_data[y, x-1] == 0 or self.map_data[y, x+1] == 0):
                        frontiers.append((x, y))
        
        return frontiers

    def select_closest_frontier(self, frontiers, robot_pos):
        """Select the frontier closest to the robot"""
        if not frontiers or not robot_pos:
            return None
            
        # Simple distance-based selection
        rx, ry = robot_pos
        closest = min(frontiers, key=lambda f: (f[0]-rx)**2 + (f[1]-ry)**2)
        return closest

    def navigate_to_frontier(self, frontier):
        """Send navigation goal to the frontier"""
        if not frontier or self.map_info is None:
            self.get_logger().warn("Invalid frontier or no map info.")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Convert map coordinates to world coordinates
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        
        goal_pose.pose.position.x = frontier[0] * resolution + origin.x
        goal_pose.pose.position.y = frontier[1] * resolution + origin.y
        goal_pose.pose.orientation.w = 1.0  # Default orientation
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f"Navigating to frontier at ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")

    def goal_response_callback(self, future):
        """Handle goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Navigation goal was rejected!")
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {str(e)}")

    def explore(self):
        """Main exploration routine with stuck detection and unstuck movement"""
        if self.map_data is None:
            self.get_logger().warn("Waiting for map data...")
            return
            
        robot_pos = self.get_robot_position()
        if not robot_pos:
            self.get_logger().warn("Cannot get robot position")
            return

        if self.is_stuck(robot_pos):
            self.get_logger().warn("Robot is stuck! Attempting recovery...")
            self.unstuck_robot()

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers found - exploration complete?")
            return
            
        target = self.select_closest_frontier(frontiers, robot_pos)
        if target:
            self.navigate_to_frontier(target)
        else:
            self.get_logger().warn("No valid frontier found!")


def main(args=None):
    rclpy.init(args=args)
    explorer = SimpleFrontierExplorer()
    
    try:
        rclpy.spin(explorer)
    except (KeyboardInterrupt, ROSInterruptException):
        explorer.get_logger().info("Exploration stopped")
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

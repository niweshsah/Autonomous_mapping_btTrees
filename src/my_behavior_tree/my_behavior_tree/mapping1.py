import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import py_trees
import py_trees.behaviours
import py_trees.composites
import py_trees.common

class CheckObstacle(py_trees.behaviour.Behaviour):
    """
    Checks for obstacles using LaserScan data. Returns SUCCESS if no obstacle
    is closer than a defined threshold, otherwise FAILURE.
    """
    def __init__(self, name="CheckObstacle"):
        super(CheckObstacle, self).__init__(name)
        self.scan_data = None

    def initialise(self):
        # Reset data if necessary at each tick.
        self.scan_data = None

    def update(self):
        if self.scan_data is None:
            # No data yet, assume path is clear.
            return py_trees.common.Status.SUCCESS

        threshold = 0.7  # meters, adjust based on your needs
        if min(self.scan_data.ranges) < threshold:
            self.logger.debug("Obstacle too close!")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

class ExploreForward(py_trees.behaviour.Behaviour):
    """
    Drives the TurtleBot forward for exploration. Publishes Twist commands
    continuously and returns RUNNING.
    """
    def __init__(self, node, name="ExploreForward"):
        super(ExploreForward, self).__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        twist = Twist()
        twist.linear.x = 0.2  # Constant forward speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.node.get_logger().info("Exploring forward...")
        return py_trees.common.Status.RUNNING

class Turn(py_trees.behaviour.Behaviour):
    """
    Commands the TurtleBot to turn in place for a specified duration.
    This helps the robot to reorient and continue exploring.
    """
    def __init__(self, node, name="Turn"):
        super(Turn, self).__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.turn_duration = 1.5  # seconds to perform a turn
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Turn at 0.5 rad/s
        self.publisher.publish(twist)
        self.node.get_logger().info("Turning to avoid obstacle...")
        if time.time() - self.start_time >= self.turn_duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class MappingBTNode(Node):
    """
    A ROS2 node that uses a behavior tree to drive exploration.
    In parallel, you should run a SLAM node (e.g., gmapping or Cartographer)
    to convert sensor data into a map.
    """
    def __init__(self):
        super().__init__('mapping_bt_executor')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Create behavior tree nodes.
        self.check_obstacle = CheckObstacle()
        self.explore_forward = ExploreForward(self)
        self.turn = Turn(self)

        # Build a sequence: if no obstacle is detected, keep exploring.
        explore_sequence = py_trees.composites.Sequence("ExploreSequence")
        explore_sequence.add_child(self.check_obstacle)
        explore_sequence.add_child(self.explore_forward)

        # Use a selector: if exploring fails (due to an obstacle), then turn.
        self.root = py_trees.composites.Selector("RootSelector")
        self.root.add_child(explore_sequence)
        self.root.add_child(self.turn)

        # Timer to tick the behavior tree regularly.
        self.create_timer(0.5, self.tick_tree)

    def scan_callback(self, msg):
        # Update the latest LaserScan data for the obstacle checker.
        self.check_obstacle.scan_data = msg

    def tick_tree(self):
        # Tick the tree once per timer callback.
        self.root.tick_once()

def main(args=None):
    rclpy.init(args=args)
    node = MappingBTNode()

    # In your launch setup, run a SLAM node concurrently (e.g., gmapping)
    # to build a map from the incoming sensor data.

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mapping BT Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

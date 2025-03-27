#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import time

# ---- Condition Node: Check Battery Level ----
class CheckBattery(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckBattery"):
        super(CheckBattery, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key("battery", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.battery < 20:  # Low battery
            self.logger.info("Battery low! Returning to charge.")
            return py_trees.common.Status.FAILURE
        self.logger.info("Battery OK.")
        return py_trees.common.Status.SUCCESS

# ---- Action Node: Move Forward ----
class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveForward", cmd_vel_pub=None):
        super(MoveForward, self).__init__(name)
        self.cmd_vel_pub = cmd_vel_pub

    def update(self):
        twist = Twist()
        twist.linear.x = 0.2  # Move forward
        self.cmd_vel_pub.publish(twist)
        self.logger.info("Moving forward.")
        return py_trees.common.Status.SUCCESS

# ---- Condition Node: Check for Obstacles ----
class CheckObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckObstacle"):
        super(CheckObstacle, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key("obstacle", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.obstacle:
            self.logger.info("Obstacle detected! Moving back.")
            return py_trees.common.Status.FAILURE  # Triggers ReturnToCharge
        return py_trees.common.Status.SUCCESS

# ---- Action Node: Return to Safety ----
class ReturnToCharge(py_trees.behaviour.Behaviour):
    def __init__(self, name="ReturnToCharge", cmd_vel_pub=None):
        super(ReturnToCharge, self).__init__(name)
        self.cmd_vel_pub = cmd_vel_pub

    def update(self):
        twist = Twist()
        twist.linear.x = -0.2  # Move backward
        self.cmd_vel_pub.publish(twist)
        self.logger.info("Avoiding obstacle by moving back.")
        time.sleep(2)  # Move back for 2 seconds
        return py_trees.common.Status.SUCCESS

# ---- ROS 2 Node Running the Behavior Tree ----
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__("turtlebot_behavior_tree")
        self.get_logger().info("Starting TurtleBot Behavior Tree...")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.create_subscription(Float32, "/battery_level", self.battery_callback, 10)

        # Blackboard (Shared memory)
        self.blackboard = py_trees.blackboard.Client(name="Blackboard")
        self.blackboard.register_key("battery", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("obstacle", access=py_trees.common.Access.WRITE)

        # Initialize default values
        self.blackboard.battery = 100  # Assume full battery at start
        self.blackboard.obstacle = False  # No obstacle initially

        # Behavior Tree Nodes
        check_battery = CheckBattery()
        check_obstacle = CheckObstacle()
        move_forward = MoveForward(cmd_vel_pub=self.cmd_vel_pub)
        return_to_charge = ReturnToCharge(cmd_vel_pub=self.cmd_vel_pub)

        # Sequence: Move if battery is OK and no obstacle
        move_sequence = py_trees.composites.Sequence("MoveSequence", memory=True)
        move_sequence.add_children([check_battery, check_obstacle, move_forward])

        # Selector: Move forward OR avoid obstacle
        root = py_trees.composites.Selector("RootSelector", memory=True)
        root.add_children([move_sequence, return_to_charge])

        # Create the behavior tree
        self.tree = py_trees.trees.BehaviourTree(root)

        # Run behavior tree loop
        while rclpy.ok():
            self.tree.tick()
            time.sleep(0.5)

    # ---- Callback: Laser Scan (Obstacle Detection) ----
    def laser_callback(self, msg):
        min_distance = min(msg.ranges)  # Closest obstacle
        self.blackboard.obstacle = min_distance < 0.5  # If obstacle < 0.5m

    # ---- Callback: Battery Level ----
    def battery_callback(self, msg):
        self.blackboard.battery = msg.data  # Update battery level

# ---- Main Function ----
def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy #python library for ROS 2
from rclpy.node import Node #Node class for creating ROS 2 nodes
import py_trees #python library for behavior trees
import time #python library for time operations



class SimpleAction(py_trees.behaviour.Behaviour):
    """A simple behavior that simulates an action."""

    def __init__(self, name="SimpleAction"):
        super(SimpleAction, self).__init__(name)

    def update(self):
        self.logger.info(f"Executing {self.name}...")
        time.sleep(1)  # Simulate work
        return py_trees.common.Status.SUCCESS


class BehaviorTreeNode(Node):
    """ROS 2 Node that runs a Behavior Tree."""

    def __init__(self):
        super().__init__("behavior_tree_node")
        self.get_logger().info("Starting Behavior Tree...")

        # Create behavior tree
        root = SimpleAction("RootAction")
        tree = py_trees.trees.BehaviourTree(root)

        # Run behavior tree loop
        while rclpy.ok():
            tree.tick()
            time.sleep(1)


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

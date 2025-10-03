import rclpy
from rclpy.node import Node
import torch

# ...existing imports if needed...


class SimpleTorchNode(Node):
    def __init__(self):
        super().__init__("simple_node")
        self.get_logger().info(f"Simple torch node alive!")
        self.get_logger().info(f"Torch version: {torch.__version__}!")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTorchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

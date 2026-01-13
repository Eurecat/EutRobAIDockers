import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
import torch


class SimpleTorchNode(Node):
    def __init__(self):
        super().__init__("simple_node")
        
        self.get_logger().info(f"Simple torch node alive!")
        self.get_logger().info(f"Torch version: {torch.__version__}!")

        self.torch_version_pub : Publisher = self.create_publisher(
            String, "torch_version", 10
        )
        self.pub_torch_version()
    
    def pub_torch_version(self):
        msg = String()
        msg.data = torch.__version__
        self.torch_version_pub.publish(msg)
        self.get_logger().info(f"Published torch version: {msg.data}")

    @staticmethod
    def compute_zero_tensor(shape):
        return torch.zeros(shape)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTorchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

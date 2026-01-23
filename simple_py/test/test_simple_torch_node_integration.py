"""
Integration test for SimpleTorchNode using launch_pytest
This test launches the actual node and verifies it publishes torch_version messages
"""
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from threading import Event


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Launch the simple_torch node for integration testing
    """
    simple_torch_node = launch_ros.actions.Node(
        package='simple_py',
        executable='simple_torch',
        name='simple_torch_node',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            simple_torch_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'simple_torch_node': simple_torch_node,
        }
    )


class TestSimpleTorchNodeIntegration:
    """Integration tests for SimpleTorchNode"""

    @staticmethod
    def test_node_publishes_torch_version(launch_service, proc_output):
        """Test that the node publishes torch version on startup"""
        rclpy.init()
        
        try:
            # Create a test node to subscribe
            test_node = Node('test_subscriber')
            received_messages = []
            message_event = Event()
            
            def callback(msg):
                received_messages.append(msg)
                message_event.set()
            
            # Subscribe to torch_version topic
            subscription = test_node.create_subscription(
                String,
                '/torch_version',
                callback,
                10
            )
            
            # Wait for message (with timeout)
            timeout = 10.0  # seconds
            start_time = test_node.get_clock().now()
            
            while not message_event.is_set():
                rclpy.spin_once(test_node, timeout_sec=0.1)
                elapsed = (test_node.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    break
            
            # Assert we received at least one message
            assert len(received_messages) > 0, "No torch_version message received"
            
            # Verify message format
            msg = received_messages[0]
            assert isinstance(msg, String), "Message is not of type String"
            assert len(msg.data) > 0, "Torch version string is empty"
            
            # Verify version format (e.g., "2.1.0")
            version_parts = msg.data.split('.')
            assert len(version_parts) >= 2, f"Invalid version format: {msg.data}"
            
            # Verify version is 1.7 or above
            major = int(version_parts[0])
            minor = int(version_parts[1])
            assert major > 1 or (major == 1 and minor >= 7), \
                f"Torch version {msg.data} is below required 1.7"
            
            print(f"✓ Successfully received torch version: {msg.data}")
            
        finally:
            test_node.destroy_node()
            rclpy.shutdown()

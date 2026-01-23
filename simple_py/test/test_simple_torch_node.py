import pytest
import torch
import rclpy
from std_msgs.msg import String
from simple_py.simple_torch_node import SimpleTorchNode


class TestSimpleTorchNode:
    """Test suite for SimpleTorchNode class"""
    
    def test_compute_zero_tensor_1d(self):
        """Test compute_zero_tensor with 1D shape"""
        shape = (5,)
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert torch.all(result == 0)
        assert result.dtype == torch.float32  # default dtype
    
    def test_compute_zero_tensor_2d(self):
        """Test compute_zero_tensor with 2D shape"""
        shape = (3, 4)
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert torch.all(result == 0)
    
    def test_compute_zero_tensor_3d(self):
        """Test compute_zero_tensor with 3D shape"""
        shape = (2, 3, 4)
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert torch.all(result == 0)
    
    def test_compute_zero_tensor_single_element(self):
        """Test compute_zero_tensor with single element shape"""
        shape = (1,)
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert result.item() == 0
    
    def test_compute_zero_tensor_large_shape(self):
        """Test compute_zero_tensor with large shape"""
        shape = (100, 100)
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert torch.all(result == 0)
        assert result.numel() == 10000
    
    def test_compute_zero_tensor_empty_tuple(self):
        """Test compute_zero_tensor with scalar (empty tuple shape)"""
        shape = ()
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert result.item() == 0
    
    @pytest.mark.parametrize("shape", [
        (5,),
        (3, 4),
        (2, 3, 4),
        (10, 20, 30, 40),
    ])
    def test_compute_zero_tensor_parametrized(self, shape):
        """Parametrized test for multiple shapes"""
        result = SimpleTorchNode.compute_zero_tensor(shape)
        
        assert isinstance(result, torch.Tensor)
        assert result.shape == shape
        assert torch.all(result == 0)


class TestSimpleTorchNodeROS:
    """Test suite for SimpleTorchNode ROS functionality"""
    
    @classmethod
    def setup_class(cls):
        """Initialize ROS once for all tests in this class"""
        rclpy.init()
    
    @classmethod
    def teardown_class(cls):
        """Shutdown ROS after all tests"""
        rclpy.shutdown()
    
    def test_node_initialization(self):
        """Test that the node initializes correctly"""
        node = SimpleTorchNode()
        
        assert node.get_name() == "simple_node"
        assert node.torch_version_pub is not None
        
        node.destroy_node()
    
    def test_publisher_exists(self):
        """Test that torch_version publisher is created"""
        node = SimpleTorchNode()
        
        # Check that publisher exists and has correct topic
        publishers = node.get_publisher_names_and_types_by_node(
            "simple_node", 
            node.get_namespace()
        )
        
        topic_names = [name for name, _ in publishers]
        assert "/torch_version" in topic_names
        
        node.destroy_node()
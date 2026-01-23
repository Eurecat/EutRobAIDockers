# Testing Guide for simple_py Package

This directory contains unit and integration tests for the `simple_py` package, demonstrating ROS 2 testing with PyTorch integration using pytest.

## Test Structure

The package implements a **two-layer testing approach**:

```
┌──────────────────────────────────────────────────────┐
│  Integration Tests                                    │
│  (test_simple_torch_node_integration.py)             │
│  - Launch actual ROS nodes                           │
│  - End-to-end message publication validation         │
│  - Uses launch_pytest framework                      │
└────────────────┬─────────────────────────────────────┘
                 │
                 │ Validates
                 ▼
┌──────────────────────────────────────────────────────┐
│  Unit Tests                                           │
│  (test_simple_torch_node.py)                         │
│  - Pure static method testing                        │
│  - PyTorch tensor operations                         │
│  - No ROS dependencies needed                        │
└──────────────────────────────────────────────────────┘
```

## Test Files

1. **`test_simple_torch_node.py`** (Pure Unit Tests)
   - Tests the static `compute_zero_tensor()` method
   - No ROS initialization required
   - Fast, isolated PyTorch validation
   - Various tensor shapes and edge cases

2. **`test_simple_torch_node_integration.py`** (ROS Integration Tests)
   - Launches the actual `simple_torch` node
   - Validates message publication on `/torch_version` topic
   - Uses `launch_pytest` for node lifecycle management
   - End-to-end ROS communication validation

3. **`conftest.py`** (Test Configuration)
   - Configures Python environment for AI/PyTorch dependencies
   - Ensures proper venv site-packages are loaded
   - Provides consistent test environment setup

## Key Testing Concepts

### pytest Markers

```python
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Launch nodes for integration testing
```

- `@pytest.mark.launch_test`: Marks tests that require ROS node launching
- `@launch_testing.markers.keep_alive`: Keeps nodes running during test execution

### Environment Setup (conftest.py)

The `conftest.py` automatically configures the Python environment to use the AI venv with PyTorch:

```python
VENV_PATH = os.environ.get("AI_VENV", "/opt/ros_python_env")
```

This ensures tests have access to the same PyTorch installation as the running node.

## Running the Tests

### Run all tests
```bash
cd /path/to/workspace
colcon build --packages-select simple_py
colcon test --packages-select simple_py
colcon test-result --verbose
```

### Run with detailed output
```bash
colcon test --packages-select simple_py --event-handlers console_direct+ --pytest-args '-v'
```

### Run specific test file
```bash
colcon test --packages-select simple_py --pytest-args 'test/test_simple_torch_node.py -v'
```

### Run specific test function
```bash
colcon test --packages-select simple_py --pytest-args 'test/test_simple_torch_node.py::TestSimpleTorchNode::test_compute_zero_tensor_1d -v'
```

## Test Coverage

The test suite validates:

✅ **Pure Python/PyTorch Logic**
- Tensor creation and shape validation
- Zero tensor computation correctness
- Various dimensional cases (1D, 2D, 3D)
- Edge cases (single element, large tensors)

✅ **ROS Integration**
- Node launches successfully
- Topic registration (`/torch_version`)
- Message publication
- PyTorch version reporting
- End-to-end ROS communication

## Additional Resources

- [ROS 2 Testing with Python](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Python.html)
- [pytest Documentation](https://docs.pytest.org/)
- [launch_pytest](https://github.com/ros2/launch/tree/rolling/launch_pytest)
- [PyTorch Testing Best Practices](https://pytorch.org/docs/stable/testing.html)

## Summary

This test suite demonstrates **pytest-based ROS 2 testing**:

- ✅ **Layered testing**: Pure logic (unit) + ROS interface (integration)
- ✅ **PyTorch integration**: Proper venv configuration for AI dependencies
- ✅ **launch_pytest**: Node lifecycle management for integration tests
- ✅ **Pytest best practices**: Fixtures, markers, parametrization
- ✅ **Template-ready**: Can be adapted for other ROS 2 Python packages

By separating unit and integration tests, you ensure both the algorithm correctness and ROS communication are validated independently.

# EutRobAI Docker Base Images

This repository provides a **configurable Docker base image** for robotics and AI development, offering minimal and reproducible Docker setup with **PyTorch** support and choice between different ROS 2 distributions.

The purpose is to serve as a **flexible base container** for robotics and AI projects, ensuring consistency and portability across environments while allowing teams to choose between standard ROS 2 or Vulcanexus distributions.

---

## 📦 Configurable Base Image Options

### **Standard ROS 2 Jazzy + PyTorch** (Default)
- Standard **ROS 2 Jazzy Desktop Full** distribution  
- **PyTorch** for deep learning models
- General-purpose robotics development with AI capabilities

### **ROS 2 Vulcanexus (Jazzy) + PyTorch** (with `--vulcanexus` flag)
- **ROS 2 Vulcanexus (Jazzy)** as the robotics middleware
- **PyTorch** for deep learning models  
- Optimized for multimodal perception pipelines:
  - **Sound perception** (VAD, ASR)
  - **Visual perception** (entity/person detection, skeletons, posture, gestures, faces, gaze)
  - **Multimodal knowledge integration** (person manager, identity tracking)


<p align="center">
  <img src="a05eb063-7279-4bdb-88d5-3ed93e5b2141.png" width="720"/>
</p>

---

## 🚀 Quick Start

### 1. Clone the repository
```bash
git clone git@github.com:Eurecat/EutRobAIDockers.git
cd EutRobAIDockers/Docker
```

### 2. Build your desired base image

#### For Standard ROS 2 Jazzy + PyTorch (default):
```bash
./build_container.sh
```
This produces the image: **eut_ros_jazzy_torch:latest**

#### For ROS 2 Vulcanexus (Jazzy) + PyTorch:
```bash
./build_container.sh --vulcanexus
```
This produces the image: **eut_ros_vulcanexus_torch:jazzy**

### 3. Optional: Force a clean rebuild

Add the `--clean-rebuild` flag to any build command:
```bash
./build_container.sh --clean-rebuild
# or
./build_container.sh --vulcanexus --clean-rebuild
```

## 🔧 Build Configuration

The single `Dockerfile` uses build arguments to configure the base image:

- **Default**: `osrf/ros:jazzy-desktop-full` (Standard ROS 2 Jazzy)
- **With `--vulcanexus`**: `eprosima/vulcanexus:jazzy-desktop` (Vulcanexus Jazzy)

The build script automatically selects the appropriate image name based on the chosen base.

## Launch

### Option A: Deployment

As simple as...
   ```bash
   docker compose up [--force-recreate]
   ```
... within `Docker/` folder

`--force-recreate` option suggested to avoid reusing cached stopped container.

### Option B: DevContainer (Development)

In this case you need to specify a different docker compose file:
   ```bash
   docker compose -f dev-docker-compose.yaml up [--force-recreate]
   ```
... within `Docker/` folder


Within VS Code editor, make sure you have installed extension DevContainer, press `ctrl+shit+P` (command option) and search for "_Dev Containers: Open Folder in Container..._". From there you can select the folder Docker/DevContainer and the stack will launch in development mode (no node will be automatically started).

## 🧪 Testing

This repository includes **reference test implementations** in both `simple_cpp` and `simple_py` packages, serving as templates for testing ROS 2 nodes with AI/ML integration.

### Test Structure

Both packages implement a **layered testing approach**:

- **`simple_cpp`**: GoogleTest-based unit + ROS integration tests
  - Pure C++ algorithm tests (no ROS dependencies)
  - ROS node tests with domain isolation for parallel execution
  - Actions, services, and parameter validation
  - See [simple_cpp/test/README.md](simple_cpp/test/README.md) for comprehensive guide

- **`simple_py`**: pytest-based unit + ROS integration tests
  - Pure PyTorch logic tests (static methods)
  - ROS node integration tests using `launch_pytest`
  - Environment setup for AI venv (PyTorch dependencies)
  - See [simple_py/test/README.md](simple_py/test/README.md) for details

### Running Tests Locally

Run all tests across both packages:
```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --pytest-args '-v'
colcon test-result --all --verbose
```

Run tests for a specific package:
```bash
colcon test --packages-select simple_cpp --event-handlers console_direct+
colcon test --packages-select simple_py --pytest-args '-v'
```

### 🔄 CI/CD Integration

Tests are **automatically executed** on every push and pull request via [GitHub Actions](.github/workflows/docker-build.yml):

1. **Build**: Docker image is built with the configured ROS distribution
2. **Test**: Both `simple_cpp` and `simple_py` tests run inside the container
3. **Report**: Test results are collected and published as GitHub Actions artifacts
4. **Deploy**: On successful tests (main branch), image is pushed to Docker Hub

**Workflow Highlights**:
- JUnit XML test reports generated for visualization
- Test badge generation showing pass/fail status
- Artifacts include test results, logs, and summary
- Automated Docker Hub deployment with tagged images

See the [workflow file](.github/workflows/docker-build.yml) for implementation details.

### Notes
Please note that launching the stack might involve launch of GUI application from docker, therefore make sure in the current active session in the host you have given at least once the following command to make sure permissions are given.

```bash
xhost +local:docker
```

### Acknowledgements
For the testing part (especially cpp part), this repository has taken inspiration from this [amazing workshop](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/) from ROSCON ES 2025 edition.



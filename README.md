# EutRobAI Docker Base Images

This repository provides **multiple base Docker images** for robotics and AI development, offering minimal and reproducible Docker setups for various robotics middleware configurations with **PyTorch** support.

The purpose is to serve as a collection of **base containers** for different robotics and AI projects, ensuring consistency and portability across environments and enabling teams to choose the most appropriate base for their specific needs.

---

## 📦 Available Base Images

### 1. **ROS 2 Vulcanexus (Jazzy) + PyTorch**
- **ROS 2 Vulcanexus (Jazzy)** as the robotics middleware
- **PyTorch** for deep learning models
- Optimized for multimodal perception pipelines:
  - **Sound perception** (VAD, ASR)
  - **Visual perception** (entity/person detection, skeletons, posture, gestures, faces, gaze)
  - **Multimodal knowledge integration** (person manager, identity tracking)

### 2. **ROS 2 Jazzy + PyTorch**
- Standard **ROS 2 Jazzy Desktop** distribution
- **PyTorch** for deep learning models
- General-purpose robotics development with AI capabilities


<p align="center">
  <img src="a05eb063-7279-4bdb-88d5-3ed93e5b2141.png" width="720"/>
</p>

---

## 🚀 Quick Start

### 1. Clone the repository
```bash
git clone https://github.com/Eurecat/EutRobAiDockers
cd EutRobAiDockers/docker
```

### 2. Choose and build your desired base image

#### For ROS 2 Vulcanexus (Jazzy) + PyTorch:
```bash
./build_vulcanexus_torch.sh
```
This produces the image: **eut_ros_vulcanexus_torch:jazzy**

#### For ROS 2 Jazzy + PyTorch:
```bash
./build_ros2_jazzy.sh
```
This produces the image: **eut_ros2_jazzy:latest**

### 3. Optional: Force a clean rebuild

For any image, add the `--clean-rebuild` flag:
```bash
./build_vulcanexus_torch.sh --clean-rebuild
# or
./build_ros2_jazzy.sh --clean-rebuild
```

## 🏗️ Repository Structure

```
EutRobAiDockers/
├── docker/
│   ├── Dockerfile.VulcanexusTorch     # Vulcanexus Jazzy + PyTorch
│   ├── build_vulcanexus_torch.sh      # Build script for Vulcanexus
│   ├── Dockerfile.ROS2Jazzy           # Standard ROS2 Jazzy + PyTorch  
│   ├── build_ros2_jazzy.sh            # Build script for ROS2 Jazzy
│   └── [future Dockerfiles...]        # Additional base images
├── README.md
└── LICENSE
```

## 🔧 Adding New Base Images

To add a new base image configuration:

1. Create a new `Dockerfile.<YourImageName>` in the `docker/` folder
2. Create a corresponding `build_<your_image_name>.sh` script
3. Update this README with the new option
4. Ensure the build script references the correct Dockerfile with the `-f` flag

## 🔗 Related Projects

- [🔗 Cognitive Robotics Brainstorming](https://eurecatcloud.sharepoint.com/sites/robotics/Shared%20Documents/10%20Cognitive%20Robotics/Brainstorming)
- [EutEntityDetection](https://github.com/Eurecat/EutEntityDetection) - Example project using these base images


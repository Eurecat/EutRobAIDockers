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
   docker compose up
   ```
... within `Docker/` folder

Will start camera publisher and (simple) rgb perception annotation on `/entities/detected` topic.

If you want to avoid to start automatically the camera just "scale down" to zero the respective service:

   ```bash
   docker compose up --scale eut_vision_camera=0
   ```

### Option B: DevContainer (Development)

Within VS Code editor, make sure you have installed extension DevContainer, press `ctrl+shit+P` (command option) and search for "_Dev Containers: Open Folder in Container..._". From there you can select the folder Docker/DevContainer and the stack will launch in development mode (no node will be automatically started).

### Notes
Please note that launching the stack might involve launch of GUI application from docker, therefore make sure in the current active session in the host you have given at least once the following command to make sure permissions are given.

```bash
xhost +local:docker
```


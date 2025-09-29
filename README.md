# ROS 2 Vulcanexus (Jazzy) + PyTorch Base Image

This repository provides a **minimal and reproducible Docker setup** to bootstrap a multimodal perception pipeline, integrating **sound, vision, and people/face subsystems** on top of **ROS 2 Vulcanexus (Jazzy)** with **PyTorch** support.  

The purpose is to serve as a **base container** for higher-level multimodal cognitive robotics development, ensuring consistency and portability across environments.

---

## 📦 Features

- **ROS 2 Vulcanexus (Jazzy)** as the robotics middleware  
- **PyTorch** for deep learning models  
- Designed as a **base container** for multimodal perception pipelines:
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
git clone https://github.com/Eurecat/EutRosVulcanexusTorch
cd EutRosVulcanexusTorch/docker
```

### 2. Build the Docker image

Simply run:
```bash
./build_container.sh
```

The script automatically sets up the build environment and produces the image:

eut_ros_vulcanexus_torch:jazzy

### 3. Optional: Force a clean rebuild

```bash
./build_container.sh --clean-rebuild
```

[🔗 Cognitive Robotics Brainstorming](https://eurecatcloud.sharepoint.com/sites/robotics/Shared%20Documents/10%20Cognitive%20Robotics/Brainstorming)

🔮 Next Steps

Starting to work on https://github.com/Eurecat/EutEntityDetection


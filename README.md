# EutRobAI Docker Base Images

[![Build Status](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml/badge.svg?branch=jazzy)](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml?query=branch%3Ajazzy)

**Standard ROS 2:**
[![Tests (Standard)](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/Eurecat/EutRobAIDockers/badges/jazzy/standard/test-badge.json)](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml)
[![Coverage (Standard)](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/Eurecat/EutRobAIDockers/badges/jazzy/standard/coverage-badge.json)](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml)

**Vulcanexus:**
[![Tests (Vulcanexus)](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/Eurecat/EutRobAIDockers/badges/jazzy/vulcanexus/test-badge.json)](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml)
[![Coverage (Vulcanexus)](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/Eurecat/EutRobAIDockers/badges/jazzy/vulcanexus/coverage-badge.json)](https://github.com/Eurecat/EutRobAIDockers/actions/workflows/docker-build.yml)

## What This Repository Does

**EutRobAIDockers** provides the foundational Docker base images for the entire EutPerceptionStack. It serves as the common container base for all perception modules, ensuring consistent development and deployment environments across the stack. All other repositories (EutHRIFaces, EutEntityDetection, EutHRIHumanBody, EutPersonManager, eut_speech_audio_processing) build their containers on top of these base images.

<p align="center">
  <img src="Docker/imgs/perceptionstack_diagram.jpeg" alt="Base Docker Architecture" width="800"/>
</p>

## Key Characteristics

- 🐳 **Foundation for EutPerceptionStack**: All perception modules build from these base images
- 🎛️ **Dual ROS2 Distributions**: Support for both standard ROS2 Jazzy and Vulcanexus Jazzy
- 🔥 **PyTorch Integration**: Pre-configured deep learning environment with GPU support
- 🔧 **Configurable Build**: Single Dockerfile with build arguments for different configurations
- 🧪 **Reference Implementations**: Template C++ and Python packages for testing and development
- 🚀 **Production Ready**: CI/CD pipelines with automated testing and coverage reporting
- 📦 **Minimal & Reproducible**: Optimized for consistency and portability

This repository provides a **configurable Docker base image** for robotics and AI development, offering minimal and reproducible Docker setup with **PyTorch** support and choice between different ROS 2 distributions.

For Jetson Thor and other ARM64 Jetson targets, the build now uses a dedicated ARM Dockerfile that starts from an NVIDIA PyTorch base image and installs ROS 2 Jazzy on top. This avoids the generic upstream CUDA wheel flow on Jetson.

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

## 🏗️ EutPerceptionStack Architecture

This base image serves as the foundation for the complete perception stack:

**Repositories Built on EutRobAIDockers:**
- **[EutHRIFaces](https://github.com/Eurecat/EutHRIFaces)**: Face detection, recognition, gaze estimation, and visual speech activity
- **[EutEntityDetection](https://github.com/Eurecat/EutEntityDetection)**: YOLO-based object and person detection with tracking
- **[EutHRIHumanBody](https://github.com/Eurecat/EutHRIHumanBody)**: Person detection filtering and skeleton keypoint estimation
- **[EutPersonManager](https://github.com/Eurecat/EutPersonManager)**: Multi-modal person fusion (body, face, skeleton, gaze)
- **[eut_speech_audio_processing](https://github.com/Eurecat/eut_speech_audio_processing)**: Audio stream management, VAD, diarization, and ASR

All these repositories reference EutRobAIDockers as their base image and extend it with domain-specific dependencies and models.

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

#### For Jetson Thor / ARM64:
```bash
./build_container.sh --platform arm
```
This produces the image: **eut_ros_torch:jazzy** using `Docker/Dockerfile.arm` and a Jetson-compatible NVIDIA PyTorch base image.

> ⚠️ **The ARM build is designed and validated specifically for NVIDIA Jetson Thor (T5000) running JetPack 7 on Ubuntu 24.04.** It is _not_ a generic ARM64 / multi-platform build. See the [Jetson Thor / ARM target environment](#-jetson-thor--arm-target-environment) section for the exact host requirements.

### 3. Optional: Force a clean rebuild

Add the `--clean-rebuild` flag to any build command:
```bash
./build_container.sh --clean-rebuild
# or
./build_container.sh --vulcanexus --clean-rebuild
```

## 🔧 Build Configuration

The x86_64 build uses `Docker/Dockerfile` with build arguments to configure the base image:

- **Default**: `osrf/ros:jazzy-desktop-full` (Standard ROS 2 Jazzy)
- **With `--vulcanexus`**: `eprosima/vulcanexus:jazzy-desktop` (Vulcanexus Jazzy)

The ARM64 Jetson build uses `Docker/Dockerfile.arm` and forces a Jetson-compatible NVIDIA PyTorch base image, then installs standard ROS 2 Jazzy on top.

By default, the ARM path uses `nvcr.io/nvidia/pytorch:25.08-py3-igpu`. You can override that tag at build time with:

```bash
JETSON_BASE_IMAGE=<your-compatible-nvidia-image> ./build_container.sh --platform arm
```

The build script automatically selects the appropriate Dockerfile and image name based on the chosen platform.

## 🤖 Jetson Thor / ARM target environment

The `--platform arm` build path is **purpose-built for NVIDIA Jetson Thor** (Thor T5000 dev kits and equivalents) and is tightly coupled to the current Thor JetPack stack. It is **not** intended as a generic multi-arch ARM64 build, and the ROS 2 distro on ARM is fixed to **Jazzy** (no `--humble`, no `--vulcanexus`, no `--cpu` on ARM).

Reference host configuration this image is built and tested against:

| Component                | Value                                                  |
| ------------------------ | ------------------------------------------------------ |
| Board                    | NVIDIA Jetson Thor (T5000, `aarch64`)                  |
| L4T / BSP                | **R38.4.0** (`nvidia-l4t-core 38.4.0-20251230160601`)  |
| JetPack                  | **JetPack 7** (Thor GA, L4T R38.x family)              |
| Host OS                  | **Ubuntu 24.04 LTS (noble)**, kernel `6.8.12-tegra`    |
| NVIDIA driver / CUDA     | Driver `580.00` / CUDA `13.0` (reported by `nvidia-smi`) |
| Container PyTorch base   | `nvcr.io/nvidia/pytorch:25.08-py3` (noble, Python 3.12) |
| ROS 2 distro in image    | **Jazzy Desktop** (Ubuntu 24.04 codename `noble`)      |
| ROS Python venv          | `/opt/ros_python_env` (Python 3.12, system-site-packages, exposes torch + CUDA from the base) |

Why the constraints:

- The Thor JetPack 7 / L4T R38 stack only ships GPU-enabled CUDA / cuDNN / TensorRT user-space libraries against **Ubuntu 24.04 (noble)**. The `Dockerfile.arm` therefore _must_ be layered on a noble-based NVIDIA PyTorch image; mixing in a `jammy` (22.04) base would lose Thor GPU access.
- ROS 2 Jazzy is the distro that natively ships for Ubuntu 24.04 / noble. ROS 2 **Humble is Ubuntu 22.04 (jammy) only** and would require a jammy base image, which currently has no Thor-compatible NVIDIA PyTorch container with matching CUDA libraries — so `--platform arm --humble` is intentionally rejected by `build_container.sh`.
- For the same reason, `--vulcanexus` and `--cpu` are also rejected on ARM: the Vulcanexus images are not Jetson-tuned, and CPU-only would defeat the purpose of building on Thor.

Assumed host setup before building:
- JetPack 7 flashed and `nvidia-l4t-*` packages installed on the host.
- Docker installed with the **`nvidia` runtime** registered (`docker info | grep -i runtime` should list `nvidia`); this repo's `docker-compose.yaml` selects it via `DOCKER_RUNTIME=nvidia` in `.env`.
- Network access to `nvcr.io` to pull `nvcr.io/nvidia/pytorch:25.08-py3` on the first build.

Validation that the image actually has GPU access from inside the container:

```bash
docker run --rm --runtime nvidia eut_ros_torch:jazzy \
  bash -lc 'source /opt/ros_python_env/bin/activate && \
            python -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"'
```

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

### 🔍 Local CI/CD Verification

Before pushing changes, you can verify the entire CI/CD pipeline locally using the **verification script** inside the Docker container:

```bash
# Inside the container, run with all packages
/quick_test_coverage.sh --all

# Or specify packages explicitly
/quick_test_coverage.sh --cpp simple_cpp --python simple_py

# Clean build before testing
/quick_test_coverage.sh --all --clean
```

This script mirrors the GitHub Actions workflow and provides:
- ✅ Build validation with coverage instrumentation
- 🧪 Test execution with detailed results
- 📊 Coverage report generation (HTML + LCOV)
- 📈 Coverage statistics summary

See `--help` for all options. This ensures your changes will pass CI before pushing.

**Coverage Reports Location**:
- Python: `simple_py/htmlcov/index.html`
- C++: `build/simple_cpp/coverage_html/index.html`

**Note**: For C++ coverage, the package must be built with coverage flags:
```bash
colcon build --packages-select simple_cpp --cmake-clean-cache \
  --cmake-args -DCMAKE_CXX_FLAGS='--coverage' \
               -DCMAKE_C_FLAGS='--coverage' \
               -DCMAKE_EXE_LINKER_FLAGS='--coverage'
colcon test --packages-select simple_cpp
```

### �🔄 CI/CD Integration

Tests are **automatically executed** on every push and pull request via [GitHub Actions](.github/workflows/docker-build.yml).

**Package Configuration**: The workflow uses centralized package definitions in [`.github/workflows/docker-build.yml`](.github/workflows/docker-build.yml#L18-L20) and [`Docker/ci_cd_coverage.sh`](Docker/ci_cd_coverage.sh#L4-L9), making it easy to adapt for your own packages.

**For detailed instructions on setting up this CI/CD pipeline in your own repository, see [CI/CD Setup Guide](CI_CD_SETUP.md).**

1. **Build**: Docker image is built with the configured ROS distribution
2. **Test**: Both `simple_cpp` and `simple_py` tests run inside the container
3. **Coverage**: Code coverage reports generated for both packages
4. **Report**: Test results and coverage are collected and published as GitHub Actions artifacts
5. **Deploy**: On successful tests (main branch), image is pushed to Docker Hub

**Workflow Highlights**:
- JUnit XML test reports generated for visualization (Dorny test-reporter)
- Code coverage reports (HTML + lcov) for both Python and C++ packages
- Coverage summary table with PR comments (on pull requests)
- Test and coverage badge generation (JSON artifacts)
- Artifacts include test results, coverage reports, logs, and summaries
- Automated Docker Hub deployment with tagged images

See the [workflow file](.github/workflows/docker-build.yml) for implementation details.

### Notes
Please note that launching the stack might involve launch of GUI application from docker, therefore make sure in the current active session in the host you have given at least once the following command to make sure permissions are given.

```bash
xhost +local:docker
```

### Acknowledgements
For the testing part (especially cpp part), this repository has taken inspiration from this [amazing workshop](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/) from ROSCON ES 2025 edition.



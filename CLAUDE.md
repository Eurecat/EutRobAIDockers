# EutRobAIDockers — agent guide

Base Docker images (ROS 2 + PyTorch + venv) that every EutPerceptionStack component builds on.
Build this first. No perception code: `simple_py` / `simple_cpp` are template packages that exercise
the test/coverage toolchain. Component of EutPerceptionStack (stack-wide guide: `../../CLAUDE.md` when
cloned under `stack/`). `AGENTS.md` is a symlink to this file.

## Layout

```
Docker/Dockerfile        x86_64: osrf/ros:jazzy-desktop-full (or eprosima/vulcanexus with --vulcanexus; humble with --humble)
Docker/Dockerfile.arm    Jetson Thor: nvcr.io/nvidia/pytorch:25.08-py3 / Orin: dustynv/pytorch:2.7-r36.4.0-cu128-24.04,
                         + ROS 2 Jazzy (JETSON_BASE_IMAGE overrides; JETSON_TARGET=thor|orin picks the board)
Docker/build_container.sh, docker-compose.yaml, dev-docker-compose.yaml, requirements*.txt,
       ci_cd_coverage.sh, quick_test_coverage.sh, DevContainer/
simple_py/, simple_cpp/  template packages + testing tutorials (simple_cpp/test/*.md)
.github/workflows/       Docker build CI
CI_CD_SETUP.md           CI/CD pipeline
plans/                   dev notes
```

## Build / test

```bash
cd Docker && ./build_container.sh [--vulcanexus|--humble|--cpu|--clean-rebuild]   # eut_ros_torch:<distro> (…_vulcanexus_torch, _cpu)
./build_container.sh --platform arm                                              # eut_ros_torch_arm:jazzy (Thor) / eut_ros_torch_arm_orin:jazzy (Orin)
./quick_test_coverage.sh        # or inside the container: colcon test --packages-select simple_py simple_cpp
```

Images provide: venv `/opt/ros_python_env` (Python 3.12 on Jazzy, 3.10 on Humble, `uv`),
both `rmw_cyclonedds_cpp` and `rmw_fastrtps_cpp`, workspace `/workspace`.

## Traps

- ARM is Jetson Thor (JetPack 7, L4T R38) or Orin (JetPack 6, L4T R36), detected from
  `/etc/nv_tegra_release`; Orin images get the `_arm_orin` suffix and the base sets
  `ENV JETSON_TARGET` (+ `/opt/jetson_constraints.txt` on Orin) for component Dockerfiles. `--platform arm` rejects
  `--humble`, `--vulcanexus` and `--cpu` on purpose.
- A change here rebuilds every downstream image; component Dockerfiles take it as `BASE_IMAGE`.
- Check GPU access in the ARM image with the torch one-liner in the README ("Jetson Thor").

## Commits

Plain human sentences describing the change, no prefixes. No AI attribution: no
`Co-Authored-By` trailer, no "Generated with" line.

# Installation

The workshop targets **Ubuntu 22.04 + ROS 2 Humble**.

## 1. ROS 2 Humble

Install ROS 2 Humble using the official ROS 2 instructions, then:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git cmake build-essential python3-colcon-common-extensions \
  ros-humble-cv-bridge ros-humble-tf-transformations python3-opencv \
  libglm-dev libsdl2-dev libfreetype6-dev
```

## 2. Stonefish

This workshop uses the upstream Stonefish simulator by **Patryk Cieślak**:

```bash
cd ~
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

Stonefish requires a GPU/driver supporting OpenGL 4.3 or newer.

## 3. stonefish_ros2

Use Patryk Cieślak's upstream ROS 2 interface:

```bash
mkdir -p ~/roscon_ws/src
cd ~/roscon_ws/src
git clone https://github.com/patrykcieslak/stonefish_ros2.git
```

Stonefish and `stonefish_ros2` must be version-compatible. The exact tested
commit/tag will be pinned in the final workshop release.

## 4. Workshop repository

```bash
cd ~/roscon_ws/src
git clone <THIS_REPOSITORY_URL> roscon_stonefish_workshop

cd ~/roscon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

There is **no `cola2_stonefish` dependency**. Workshop scenarios, scripts and
the BlueBoat workshop geometry live in this repository.

## 5. Verify the BlueBoat exercise

```bash
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py
```

You should see the surface vehicle and the red goal cube.

In another sourced terminal:

```bash
ros2 topic list
```

## 6. Reconstruction tools

COLMAP/GLOMAP are optional pre-work. The workshop includes scripts for image
extraction and reconstruction. A known-good image dataset will be supplied with
the final BlueROV2 environment.

See `PRE_WORK.md` and `TROUBLESHOOTING.md`.

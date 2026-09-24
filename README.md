# Underwater Robotics with ROS and Stonefish: Simulate, Explore!

Hands-on material for the **ROSCon UK 2026** workshop.

**Presenter:** Michele Grimaldi — Heriot-Watt University

This repository introduces marine robotics with ROS 2 and
[Stonefish](https://stonefish.readthedocs.io/en/latest/) through a progressive
set of practical exercises:

**surface-vehicle control → underwater control → survey planning → image
acquisition → 3D reconstruction**

> **Workshop status**
>
> The BlueBoat exercise and BlueROV2 control stack are included. The final
> BlueROV2 survey environment and the associated known-good reconstruction
> dataset will be added before the workshop.

---

## Workshop overview

The workshop is designed as a roughly three-hour hands-on session.

Participants first explore the dynamics and sensors of marine robots in
Stonefish. They then implement a simple controller for a BlueBoat, move to a
provided low-level controller for a BlueROV2, design an underwater survey
trajectory, record camera data, and reconstruct the surveyed scene using
COLMAP or GLOMAP.

The emphasis is on the complete robotics pipeline rather than on any single
algorithm:

```text
ROS 2 + Stonefish
       |
       v
marine robot simulation
       |
       +--> BlueBoat --> participant PID --> red goal
       |
       `--> BlueROV2 --> provided PID --> participant survey planner
                                      |
                                      v
                                  RGB images
                                      |
                                      v
                              COLMAP / GLOMAP
                                      |
                                      v
                              3D reconstruction
```

---

## Learning objectives

By the end of the workshop, participants should be able to:

- launch and inspect a marine robot simulation in Stonefish;
- discover and inspect ROS 2 sensor and actuator interfaces;
- understand why marine-vehicle dynamics differ from ideal kinematic robots;
- implement and tune a simple goal-reaching controller for a surface vehicle;
- understand multi-thruster allocation on an underwater robot;
- design a survey trajectory with image overlap and reconstruction in mind;
- record image data with ROS 2;
- turn a simulated survey into a sparse 3D reconstruction.

No previous marine-robotics experience is required. Basic familiarity with
ROS 2 topics, nodes, launch files, and Python is useful.

---

## Tested platform

The workshop deliberately has a small simulator dependency chain:

```text
ROS 2 Humble
   + Stonefish (patrykcieslak/stonefish)
   + stonefish_ros2 (patrykcieslak/stonefish_ros2)
   + this repository
```

There is **no `cola2_stonefish` dependency**.

The workshop targets:

- Ubuntu 22.04
- ROS 2 Humble
- Stonefish
- `stonefish_ros2`
- Python 3

A dedicated GPU supporting OpenGL 4.3+ is strongly recommended.

The exact tested repository URLs/commits for the Stonefish ROS integration and
workshop assets will be pinned before the public workshop release.

See [`INSTALL.md`](INSTALL.md) for installation instructions and
[`PRE_WORK.md`](PRE_WORK.md) for the participant checklist.

---

## Repository structure

```text
roscon_stonefish_workshop/
├── README.md
├── PARTICIPANT_INSTRUCTIONS.md
├── INSTALL.md
├── PRE_WORK.md
├── TROUBLESHOOTING.md
├── WORKSHOP_SCHEDULE.md
├── EQUIPMENT.md
│
├── package.xml
├── setup.py
├── setup.cfg
│
├── launch/
│   ├── blueboat_workshop.launch.py
│   ├── bluerov_controller.launch.py
│   └── bluerov2_workshop.launch.py
│
├── data/
│   ├── scenarios/
│   │   ├── blueboat_goal.scn
│   │   └── BLUEROV_ENVIRONMENT_PENDING.md
│   ├── robots/
│   │   ├── blueboat.scn
│   │   └── bluerov2_pending_assets.scn
│   └── meshes/
│       └── propeller.obj
│
├── roscon_stonefish_workshop/
│   ├── blueboat_pid.py
│   ├── bluerov_pid.py
│   ├── survey_planner.py
│   ├── thruster_adapter.py
│   └── control_utils.py
│
├── exercises/
│   ├── 01_explore_the_blueboat.md
│   ├── 02_understand_the_bluerov2.md
│   ├── 03_survey_challenge.md
│   └── 04_3d_reconstruction.md
│
├── reconstruction/
│   ├── scripts/
│   │   ├── extract_images_from_bag.py
│   │   ├── run_colmap.sh
│   │   └── run_glomap.sh
│   └── reference_dataset/
│
└── scripts/
    ├── blueboat_pid_solution.py
    ├── survey_planner_solution.py
    └── bluerov2_logitechF310teleop_original.py
```

---

## Installation

Start with the full instructions in [`INSTALL.md`](INSTALL.md).

Once the dependencies and workshop workspace are available:

```bash
cd ~/roscon_ws
source /opt/ros/humble/setup.bash

# Source the workspace containing Stonefish ROS 2 / COLA2 packages.
# source <stonefish_ros_workspace>/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

Verify that the required simulator packages are visible:

```bash
ros2 pkg prefix stonefish_ros2
```

---

## Exercise 1 — BlueBoat PID challenge

The first environment is intentionally simple: a **BlueBoat and a red cube**.
The cube marks the navigation goal.

Launch the simulation:

```bash
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py
```

Participants work on:

```text
roscon_stonefish_workshop/blueboat_pid.py
```

Find the exercise sections with:

```bash
grep -R "TODO(participant)" roscon_stonefish_workshop/
```

The task is to complete and tune the goal-reaching controller while observing
the effect of the simulated first-order thruster dynamics.

An instructor reference implementation is available in:

```text
scripts/blueboat_pid_solution.py
```

---

## Exercise 2 — BlueROV2 control

For the underwater exercise, the low-level controller is provided.

```bash
ros2 launch roscon_stonefish_workshop bluerov_controller.launch.py
```

The control architecture is:

```text
PoseStamped
     |
     v
x/y/depth/yaw PID
     |
     v
geometry_msgs/Twist
     |
     v
8 x 4 thruster allocation
     |
     v
std_msgs/Float64MultiArray
     |
     v
/bluerov/controller/thruster_setpoints_sim
```

The allocation follows the existing BlueROV2 joystick interface used by the
workshop setup. Commands represent:

```text
[forward, side, rotation, depth]
```

and are mapped to the eight simulated thrusters and normalized to the range
expected by the simulator.

Participants therefore do **not** spend the workshop implementing an
eight-thruster low-level controller. Instead, they can focus on the survey.

---

## Exercise 3 — Survey-planning challenge

Participants edit:

```text
roscon_stonefish_workshop/survey_planner.py
```

The objective is to design a trajectory that produces a useful image sequence
for 3D reconstruction.

Things to consider include:

- image overlap;
- camera viewpoint;
- stand-off distance;
- line spacing;
- unnecessary rotations;
- mission duration;
- coverage of the target.

A lawn-mower trajectory provides a useful baseline, but participants are free
to improve it.

The final environment bounds and starting pose will be inserted once the
workshop survey scene is finalized.

---

## Exercise 4 — From ROS 2 to 3D

Participants record the camera stream during their survey.

A typical workflow is:

```bash
ros2 bag record <RGB_CAMERA_TOPIC>
```

Extract the images:

```bash
python3 reconstruction/scripts/extract_images_from_bag.py \
    <BAG_DIRECTORY> \
    <RGB_CAMERA_TOPIC> \
    my_survey/images
```

Run COLMAP:

```bash
./reconstruction/scripts/run_colmap.sh \
    my_survey/images \
    my_survey/colmap
```

or GLOMAP:

```bash
./reconstruction/scripts/run_glomap.sh \
    my_survey/images \
    my_survey/glomap
```

Participants then inspect:

- number of registered images;
- estimated camera trajectory;
- reconstruction coverage;
- disconnected components;
- missing regions;
- whether neighbouring survey lines are geometrically connected.

A known-good reference sequence will also be supplied so that everyone can
complete the reconstruction exercise even if their own survey does not
reconstruct successfully.

---

## Participant tasks vs provided components

| Component | Participant task? |
|---|---|
| Stonefish simulation | Provided |
| BlueBoat environment | Provided |
| BlueBoat PID | **Implement/tune** |
| BlueROV2 low-level PID | Provided |
| BlueROV2 thruster allocation | Provided |
| Survey trajectory | **Implement/design** |
| Image recording | **Perform** |
| ROS bag image extraction | Provided |
| COLMAP/GLOMAP scripts | Provided |
| Survey-quality analysis | **Perform** |

This division is deliberate: the workshop exposes the complete robotics
pipeline without requiring participants to implement every subsystem from
scratch.

---

## Reconstruction tools

The repository supports two Structure-from-Motion back ends:

### COLMAP

J. L. Schönberger and J.-M. Frahm,
*Structure-from-Motion Revisited*, CVPR 2016.

### GLOMAP

L. Pan, D. Barath, M. Pollefeys, and J. L. Schönberger,
*Global Structure-from-Motion Revisited*, ECCV 2024.

Sparse reconstruction is the default workshop target. Dense reconstruction is
optional if time and hardware permit.

---

## Stonefish

Stonefish is used as the marine simulation environment for the workshop.

Documentation:

https://stonefish.readthedocs.io/en/latest/

Related publication:

M. Grimaldi, P. Cieslak, E. Ochoa, V. Bharti, H. Rajani, I. Carlucho,
M. Koskinopoulou, Y. R. Petillot, and N. Gracias,
**“Stonefish: Supporting Machine Learning Research in Marine Robotics,”**
arXiv:2502.11887, 2025.

---

## Troubleshooting

See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

Before the workshop, the most important checks are:

```bash
ros2 pkg prefix stonefish_ros2
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py
```

If these work, you are ready for the required pre-work.

COLMAP/GLOMAP installation is not required for the initial simulator check.

---

## Workshop preparation

Participants should read:

1. [`PARTICIPANT_INSTRUCTIONS.md`](PARTICIPANT_INSTRUCTIONS.md)
2. [`INSTALL.md`](INSTALL.md)
3. [`PRE_WORK.md`](PRE_WORK.md)

The final BlueROV2 environment and reference reconstruction dataset will be
added before the workshop.

---

## License

Workshop code is distributed under the license specified in `package.xml`.

Third-party software, models, meshes, textures, and datasets remain subject to
their respective licenses.

---

## Acknowledgements

This workshop builds on Stonefish, ROS 2, COLA2, COLMAP, and GLOMAP and on
research in marine robotic simulation, navigation, perception, and
reconstruction.

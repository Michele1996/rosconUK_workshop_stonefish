# ROSCon UK 2026 — Stonefish Marine Robotics Workshop

**ROS 2 Humble · Stonefish 1.6 · BlueBoat Grand Prix · BlueROV2 survey and mapping**

This repository provides two independent marine robotics challenges. Each installation runs locally; no shared ROS network or common `ROS_DOMAIN_ID` is required. The BlueBoat Grand Prix includes a complete simulation circuit, a controller starter, a practice workflow and a locally scored three-attempt time trial. The BlueROV2 mapping exercise contains preliminary control, planning and reconstruction tools; its final survey scenario will be added in a later revision.

> **Public repository — please do not mirror or redistribute the complete workshop bundle.**
>
> The project is publicly available for use with the ROSCon UK 2026 workshop. Please link to the original repository instead of re-uploading or repackaging the full collection of scenarios, models, textures and event-specific materials. Independently developed controllers, policies and results may be shared subject to applicable rights and licences.
>
> **Licensing:** This is a request, not an additional restriction on third-party or open-source licences. The ROS package identifies its software licence as `Apache-2.0`, which permits redistribution under its terms. Stonefish, `stonefish_ros2`, third-party meshes and textures retain their respective licence conditions. Check the rights to bundled assets before republishing them.

**Repository status:** BlueBoat race logic has been checked with offline tests; a final graphical/propulsion check must be performed on the target Stonefish installation. The second challenge is a **work in progress**, not yet a complete ready-to-run mapping scenario.

## Contents

1. [Overview and objectives](#1-overview-and-objectives)
2. [Prerequisites and installation](#2-prerequisites-and-installation)
3. [BlueBoat circuit and scoring rules](#3-blueboat-circuit-and-scoring-rules)
4. [Practice Mode and direct thruster control](#4-practice-mode-and-direct-thruster-control)
5. [PID, vision and reinforcement learning](#5-pid-vision-and-reinforcement-learning)
6. [Official attempts and result export](#6-official-attempts-and-result-export)
7. [Track-map release](#7-track-map-release)
8. [BlueROV2 survey and 3D mapping](#8-bluerov2-survey-and-3d-mapping)
9. [Troubleshooting and verification](#9-troubleshooting-and-verification)
10. [Package structure and sharing](#10-package-structure-and-sharing)

## 1. Overview and objectives

| Challenge | Platform | Objective | Availability |
| --- | --- | --- | --- |
| **BlueBoat Grand Prix** | BlueBoat, two differential thrusters | Navigate a buoy-defined course using PID, RL, computer vision or a hybrid controller; minimise penalty-adjusted completion time. | Implemented; verify in Stonefish |
| **Underwater Survey and Mapping** | BlueROV2 | Plan an underwater survey, acquire overlapping observations and reconstruct a scene with COLMAP or GLOMAP. | Preliminary tools; final environment pending |

A local installation contains Stonefish, ROS 2 interfaces, control nodes and the BlueBoat race judge. Timed attempts are conducted independently and exported for comparison. A common network or central simulator is **not** required.

## 2. Prerequisites and installation

The target environment is **Ubuntu 22.04**, **ROS 2 Humble**, **Stonefish 1.6**, `stonefish_ros2`, Python 3 and a GPU supporting OpenGL 4.3. Install and verify Stonefish and `stonefish_ros2` before installing this package. The Grand Prix does not require COLA2.

Upstream references: [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html), [Stonefish](https://stonefish.readthedocs.io/en/latest/) and [stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2).

Optional requirements include an OpenCV build with the ArUco module (visual marker detection) and COLMAP/GLOMAP (3D reconstruction). The ROS package uses `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs` and `ament_index_python`.

Install standard ROS build tools if needed:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep unzip
```

From an existing ROS workspace with a working Stonefish installation:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Adjust this archive location if necessary.
mkdir -p /tmp/roscon_release
unzip -oq ~/Downloads/rosconUK_Stonefish_Workshop_FORMAL_README_COMPLETE.zip \
  -d /tmp/roscon_release

# Save an older installation outside the workspace source tree.
if [ -d src/roscon_stonefish_workshop ]; then
  mv src/roscon_stonefish_workshop \
    "$HOME/roscon_stonefish_workshop_previous_$(date +%Y%m%d_%H%M%S)"
fi

cp -a /tmp/roscon_release/rosconUK_workshop_stonefish \
  src/roscon_stonefish_workshop

colcon build --symlink-install --packages-select roscon_stonefish_workshop
source install/setup.bash
ros2 pkg prefix roscon_stonefish_workshop
ros2 pkg prefix stonefish_ros2
```

For a repository checkout, place the `roscon_stonefish_workshop` directory directly under `~/ros2_ws/src/` and run the same `colcon build` command. **Source ROS 2 and the workspace setup file in each new terminal.**

Offline checks, independent of a running simulator:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 -m unittest discover -s tests -v
```

Successful offline tests do not prove correct propeller direction, camera visibility, GPU rendering or dynamic response. Verify those in Stonefish before running a time trial.

## 3. BlueBoat circuit and scoring rules

The **Chiocciola v4** circuit is a point-to-point course of approximately **225.8 m**, with a nominal **6.8 m corridor width**. It contains **254 static, non-colliding yellow buoys** (127 per boundary), **12 ordered checkpoints**, **24 paired ArUco direction signs**, a green START gate and a red FINISH gate.

The route consists of a **20 m starting straight**, S-bends, an approximately **258° tightening spiral** (*chiocciola*), a **10 m straight**, a **90° elbow** (*curva a gomito*), and a **25 m finishing straight**. The course is **not** a closed lap.

The ArUco IDs are **11–22** on one side and **31–42** on the other. Their yellow headers provide a visual indication of direction. Marker detection is optional: the official checkpoint checks use the local judge's simulated ground-truth position, not computer-vision detections.

### Scoring

An official result is valid only after a forward START crossing, all **12 checkpoints in order**, and a FINISH crossing. Skipped checkpoints and cuts that bypass gates cannot create a completed result.

The adjusted time is:

\[
T_{\mathrm{adjusted}} = T_{\mathrm{elapsed}} + 8N_{\mathrm{offtrack}} + 2T_{\mathrm{outside}}.
\]

Here, `T_elapsed` is the START-to-FINISH duration in **simulation seconds**; `N_offtrack` counts separate departures from the permitted corridor; and `T_outside` is the accumulated time outside it, in simulation seconds. Each departure adds **8 s**, plus **2 s per second** spent outside. For example, a run of 150 s with two excursions totalling 5 s scores `150 + 2×8 + 2×5 = 176 s`.

The official rules are:

- **Practice is unlimited** and produces no official race result.
- **Three official attempts maximum** per team identifier and course hash. An attempt is consumed when armed, including if it is cancelled, abandoned or interrupted by a restart.
- Timing begins **only after the judge is ARMED** and the boat crosses START in the correct direction.
- CP01 through CP12 must be crossed in sequence, followed by FINISH.
- A run exceeding **1,200 simulation seconds** is a DNF; unfinished attempts do not receive a ranked time.
- The best **FINISHED** adjusted time from the three available official attempts determines the result.
- All compared runs should use an identical track, scene, judge version and physics configuration. Records generated locally are reviewable but **not tamper-proof**.

## 4. Practice Mode and direct thruster control

**Practice Mode is unrestricted training.** The boat can be driven manually or autonomously, control algorithms can be tested, and the full circuit can be explored without affecting official results. The local race judge starts in `practice` state and stays there until an official attempt is explicitly armed.

During practice, it is possible to command the two thrusters directly, tune PID or waypoint-following controllers, compare cornering speeds, detect ArUco markers, test RL policies, cross START and all 12 checkpoints repeatedly, or restart the simulator to restore the initial pose.

**No practice traversal counts toward the leaderboard.** Even a complete START → CP01 → … → CP12 → FINISH traversal is not a scored attempt unless `race_start` was run first.

### Terminal 1 — Launch the simulation with direct thruster control

To use the direct thruster demo, **disable the thruster adapter**:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch roscon_stonefish_workshop \
  blueboat_workshop.launch.py \
  team:=Team_01 \
  thruster_adapter:=false
```

This launches the BlueBoat, complete track and local race judge. Disabling the bridge avoids conflicting publishers on the physical thruster command topic.

### Terminal 2 — Run the thruster demo

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run roscon_stonefish_workshop blueboat_thruster_demo --demo
```

The demo runs a gentle straight-thrust sequence, stops, applies differential thrust and stops again. It is an actuator familiarisation example, **not** an automatic course controller.

To issue one command instead:

```bash
ros2 run roscon_stonefish_workshop \
  blueboat_thruster_demo \
  --right -0.15 \
  --left -0.15 \
  --duration 3
```

This publishes normalized motor commands of `-0.15` for 3 s and then sends stop commands. The script limits setpoints to **[-0.3, 0.3]** and duration to **15 s**. It publishes `std_msgs/msg/Float64MultiArray` at:

```text
/blueboat/controller/thruster_setpoints_sim
```

Array order:

```text
[right_thruster, left_thruster]
```

The propellers are pitched by 180°. **Confirm visually which command sign produces forward motion** before implementing an autonomous controller; do not infer the direction from the command sign alone. Stop this demo before using the velocity-command interface.

### Terminal 3 — Monitor local race status

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic echo /blueboat/race/status
```

The judge publishes JSON containing state, checkpoint progress and remaining official attempts. During practice, it should report `practice`. Check interfaces with:

```bash
ros2 topic list | grep blueboat
ros2 service list | grep /blueboat/race
```

### Switch to PID or velocity commands

Stop the direct demo and the simulator. Relaunch **with the adapter enabled** (the default):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch roscon_stonefish_workshop \
  blueboat_workshop.launch.py \
  team:=Team_01
```

In another terminal, run the supplied PID starter:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop blueboat_pid
```

Alternatively, with the PID process stopped, test the high-level command topic:

```bash
ros2 topic pub -r 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
# Ctrl+C stops publication; the adapter applies a command timeout.
```

With the adapter enabled, `geometry_msgs/msg/Twist` commands on `/blueboat/cmd_vel` are converted to right/left thruster setpoints. Avoid running multiple command publishers simultaneously.

## 5. PID, vision and reinforcement learning

### Implementing and tuning PID

The starter is `roscon_stonefish_workshop/blueboat_pid.py`. It subscribes to `/blueboat/odom` and publishes commands on `/blueboat/cmd_vel`. It includes heading-control logic, waypoint lookahead and sections marked `TODO` for implementing forward motion and tuning the controller.

**The supplied starter intentionally sets forward speed to zero**; running it unchanged will not complete the course. Implement surge control, corner-dependent speed selection and PID tuning before using an official attempt. The example reads the local course geometry from `config/blueboat_race.json`.

A separate ROS 2 control node may replace the starter, provided it publishes the same velocity command interface and does not conflict with other controllers. Available observations include odometry, IMU, GPS and the simulated forward camera. The referee's `/blueboat/race/ground_truth_odom` is the scoring source and should not be treated as an additional contest-navigation sensor unless the event rules expressly permit it.

### Optional ArUco detection

```bash
ros2 run roscon_stonefish_workshop blueboat_aruco_detector
ros2 topic echo /blueboat/aruco/ids
```

The detector relies on OpenCV's ArUco module. Visibility and decoding depend on camera viewpoint, image resolution and the rendering environment. **ArUco detection is not needed for official checkpoint validation.**

### Optional reinforcement learning

A controller can learn from **local observations without an explicit global map**: image detections of buoys/markers, heading, yaw rate, velocity and previous actions. Policies may command `/blueboat/cmd_vel` with the adapter enabled or send right/left setpoints directly with the adapter disabled.

A training reward can encourage along-track progress and successful ordered completion while penalising elapsed time, leaving the corridor and unstable commands. This reward is for policy optimisation only; **official scoring always uses the formula in Section 3**.

Repeated simulation episodes and a reset mechanism are required for training. Use Practice Mode for these episodes; reserve official attempts for completed policies. A ready-to-train Gymnasium environment or pretrained RL model is **not included**. The configuration and scene necessarily contain track geometry, so accessing those files is map-based planning rather than a strictly map-blind approach.

## 6. Official attempts and result export

Every official run is started **locally**. No central ROS server or shared IP is needed. Only activate scoring once the controller is ready.

Before starting, restart the simulator so the BlueBoat is near its initial position, enable the thruster adapter if using Twist-based control and confirm that odometry is publishing.

### Terminal 1 — Start the simulator

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch roscon_stonefish_workshop \
  blueboat_workshop.launch.py \
  team:=Team_01
```

### Terminal 2 — Run the controller

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop blueboat_pid
```

A custom PID, RL or hybrid controller may be used instead.

### Terminal 3 — Arm one official attempt

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop race_start
```

Equivalent ROS service call:

```bash
ros2 service call /blueboat/race/arm \
  std_srvs/srv/SetBool "{data: true}"
```

The judge must report **ARMED** before the START crossing. An attempt is reserved **immediately on arming** and counts toward the three-attempt limit even if subsequently abandoned.

The event sequence is **ARMED → START → CP01 … CP12 → FINISH**. The elapsed time and penalties are measured in simulation time. Monitor progress with:

```bash
ros2 topic echo /blueboat/race/status
```

For another official attempt, stop the controller and simulator, restart the scene from its initial pose and arm again. **Do not delete the results directory between runs.** To cancel an active attempt (which still consumes its slot):

```bash
ros2 run roscon_stonefish_workshop race_start --disarm
```

### Stored race records

Official records are saved locally under a team-and-course-specific directory:

```text
~/.ros/roscon_blueboat_grand_prix/Team_01/<course-hash>/attempt_01/report.json
~/.ros/roscon_blueboat_grand_prix/Team_01/<course-hash>/attempt_01/trajectory.csv
```

Reports include the course hash, attempt ID and number, outcome, elapsed time, off-track penalties, adjusted time for completed runs, checkpoint timing and the XY trajectory. DNF and abandoned attempts are not ranked, but still use an official attempt slot.

Export a submission ZIP after completing the attempts:

```bash
ros2 run roscon_stonefish_workshop race_export --team Team_01
```

Default output:

```text
~/roscon_submissions/Team_01_<course-hash>_submission.zip
```

The exporter runs offline and includes the available attempt reports and trajectories. Submit the ZIP through the event's designated collection channel.

To combine multiple submission ZIPs already collected into a directory:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 scripts/show_blueboat_leaderboard.py ~/roscon_submissions
```

Results are local, self-reported evidence. Course hashes and trajectories aid review but do **not** prevent deliberate modification on an independently controlled computer. Additional verification may be required for prizes.

## 7. Track-map release

The full-resolution overhead map and waypoint CSV are distributed **separately** from this repository's main package. The announced release point is **30 minutes after the challenge begins**. Once published in the event repository, the map can be used for waypoint-based PID, planning or RL approaches.

There is **no automatic unlock service**. The image/CSV release is manual. Because the simulator and local judge require the course geometry in `config/blueboat_race.json` and the scene files, the first 30 minutes are a **fair-play map-blind phase**, not a cryptographically enforced restriction. The README deliberately does not claim otherwise.

## 8. BlueROV2 survey and 3D mapping

The second challenge is independent of the BlueBoat race. Its objective is to plan a safe, efficient underwater survey and use recorded observations to reconstruct a scene with COLMAP or GLOMAP. Planning considerations include overlap, coverage, stand-off distance, viewing geometry and survey duration.

Included preliminary tools:

- `roscon_stonefish_workshop/bluerov_pid.py` — controller starter.
- `roscon_stonefish_workshop/survey_planner.py` — survey-planning logic to extend.
- `roscon_stonefish_workshop/control_utils.py` and `thruster_adapter.py` — control utilities.
- `reconstruction/scripts/extract_images_from_bag.py` — image extraction from ROS bag data.
- `reconstruction/scripts/run_colmap.sh` and `run_glomap.sh` — reconstruction script entrypoints; inspect the arguments before use.

The currently available launch file is a **placeholder**. The final survey world, camera topics, validated dataset and evaluation procedure have not yet been released:

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py
# Currently prints a status message; it does not launch a final mapping world.
```

Once the survey environment becomes available, inspect the actual topics and record the appropriate camera and odometry data:

```bash
ros2 topic list
ros2 bag record -o survey_bag <camera_topic> <odometry_topic>
```

Replace the bracketed topic placeholders with the discovered topic names. Do not assume a particular camera calibration, data stream or reconstruction accuracy until the final scenario is provided.

## 9. Troubleshooting and verification

| Symptom | Check |
| --- | --- |
| Stonefish aborts with `Material file not found` | Ensure the corrected `opti_blueboat` OBJ assets are installed; older propeller meshes referenced missing `.mtl` files. |
| Forward/reverse motion does not match commands | Test small direct setpoints and inspect the 180° propeller pitch and inversion settings. |
| Direct thruster demo has no effect | Launch with `thruster_adapter:=false`; the adapter otherwise competes for the setpoint topic. |
| `/blueboat/cmd_vel` has no effect | Launch with the adapter enabled (the default), publish one Twist controller at a time and check for topic activity. |
| `race_start` cannot arm | Confirm race status, odometry, initial spawn proximity (within 1.5 m) and remaining attempt slots. |
| START crossing is not scored | Check for an **ARMED** state; crossing START during practice is intentionally ignored. |
| A run fails to finish | Verify all 12 checkpoints were crossed in order and within their gate widths, before FINISH. |
| `race_export` finds no results | Check the team identifier and confirm an official attempt was armed previously. |
| ArUco signs appear incorrectly cropped | Use the centered-texture original-size box-panel textures included with this release. |
| A simulator build succeeds but runtime fails | Inspect simulator output and run Stonefish locally; the offline tests cannot validate GPU rendering or physics. |

The simulation can run at different wall-clock speeds depending on the machine; the race measures simulation time. Comparable results still require the same physics, course and judge configuration.

## 10. Package structure and sharing

```text
roscon_stonefish_workshop/
├── README.md                        # complete documentation (single README)
├── package.xml / setup.py / setup.cfg
├── launch/                          # race launch; mapping placeholder
├── config/blueboat_race.json        # course and judge geometry
├── data/
│   ├── scenarios/blueboat_race.scn  # full marked course
│   ├── robots/blueboat.scn          # BlueBoat model, sensors and thrusters
│   ├── opti_blueboat/               # model/propeller assets
│   └── textures/                    # ArUco and arrow images
├── roscon_stonefish_workshop/
│   ├── blueboat_thruster_intro.py   # direct thruster-control example
│   ├── blueboat_pid.py              # controller starter
│   ├── blueboat_race_monitor.py     # local timing and scoring
│   ├── race_engine.py / race_geometry.py / race_records.py
│   ├── race_start.py / race_export.py
│   ├── bluerov_pid.py / survey_planner.py
│   └── blueboat_aruco_detector.py
├── scripts/show_blueboat_leaderboard.py
├── reconstruction/                 # image extraction/reconstruction scripts
└── tests/                          # offline validation
```

The full workshop package is publicly accessible, but please **link to the source repository instead of mirroring the bundle**. This request does not supersede the package's Apache-2.0 software licence or any third-party licensing terms. The separate overhead map/waypoints bundle is intended for the announced 30-minute release.

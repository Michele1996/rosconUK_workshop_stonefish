# ROSCon UK 2026 — Stonefish Marine Robotics Workshop

**Release v0.8 — Fast Waypoint PID · Survey Evaluation · Custom Inspection Plans**

**ROS 2 Humble · Stonefish 1.6 · BlueBoat Grand Prix · BlueROV2 survey and mapping**

This repository provides three marine robotics challenges. Each installation runs locally; no shared ROS network or common `ROS_DOMAIN_ID` is required. The BlueBoat Grand Prix includes a complete simulation circuit, a controller starter, a practice workflow and a locally scored three-attempt time trial. The BlueROV2 exercise provides image capture, evaluation tools, and a complete map-assisted cat reference, an executable custom-inspection-plan route for bust/wreck, and optional perception/next-best-view extension points. Underwater motion and rendered images still require graphical verification.

> **Public repository — please do not mirror or redistribute the complete workshop bundle.**
>
> The project is publicly available for use with the ROSCon UK 2026 workshop. Please link to the original repository instead of re-uploading or repackaging the full collection of scenarios, models, textures and event-specific materials. Independently developed controllers, policies and results may be shared subject to applicable rights and licences.
>
> **Licensing:** This is a request, not an additional restriction on third-party or open-source licences. The ROS package identifies its software licence as `Apache-2.0`, which permits redistribution under its terms. Stonefish, `stonefish_ros2`, third-party meshes and textures retain their respective licence conditions. Check the rights to bundled assets before republishing them.

**Repository status:** BlueBoat race logic has been checked with offline tests; a final graphical/propulsion check must be performed on the target Stonefish installation. The underwater scenario includes working source files for capture and acquisition evaluation, a cat-only autonomous inspection reference, and bust/shipwreck exercise scaffolds; final runtime validation remains necessary.

## Contents

1. [Overview and objectives](#1-overview-and-objectives)
2. [Prerequisites and installation](#2-prerequisites-and-installation)

**Quick start:** see [the command guide below](#quick-start-command-guide) for the controller, three motion tests, cat reference, and bust/shipwreck custom plans.
3. [BlueBoat circuit and scoring rules](#3-blueboat-circuit-and-scoring-rules)
4. [Practice Mode and direct thruster control](#4-practice-mode-and-direct-thruster-control)
5. [PID, vision and reinforcement learning](#5-pid-vision-and-reinforcement-learning)
6. [Official attempts and result export](#6-official-attempts-and-result-export)
7. [Track-map release](#7-track-map-release)
8. [BlueROV2 survey and 3D mapping](#8-bluerov2-survey-and-3d-mapping)
9. [Survey evaluation and inspection autonomy](#9-survey-evaluation-and-inspection-autonomy)
10. [Troubleshooting and verification](#10-troubleshooting-and-verification)
11. [Package structure and sharing](#11-package-structure-and-sharing)

## Quick-start command guide

**Every terminal:** run `source /opt/ros/humble/setup.bash` and
`source ~/ros2_ws/install/setup.bash` first. Install and build the ROS package
using Section 2; the commands below assume the package is already built.

| Goal | Command | Notes |
| --- | --- | --- |
| BlueBoat practice | `ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py team:=Team_01` | Official scoring is disabled until explicitly armed. |
| BlueBoat direct thrusters | `ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py team:=Team_01 thruster_adapter:=false` | Then run `ros2 run roscon_stonefish_workshop blueboat_thruster_demo --demo` in another terminal. |
| BlueBoat official attempt | `ros2 run roscon_stonefish_workshop race_start` | Run with the simulator active; crossing START afterward begins timing and consumes one of three attempts. |
| BlueBoat submission | `ros2 run roscon_stonefish_workshop race_export --team Team_01` | Saves a ZIP with records and trajectories. |
| BlueROV2 free survey | `ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py controller:=false` | To pulse motors, run `ros2 run roscon_stonefish_workshop bluerov_thruster_demo --demo` separately. |
| BlueROV2 PID verification | `ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py controller:=true autonomy:=false` | Then run `ros2 run roscon_stonefish_workshop bluerov_waypoint_test --timeout 55`. |
| Autonomous **cat reference** | `ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py scenario:=bluerov2_autonomy_obstacles.scn controller:=true capture:=true autonomy:=true target:=cat autonomy_mode:=reference` | Map-assisted complete worked example. |
| **Bust/wreck exercises** | See Section 9.4. | Start with the unimplemented starter, then provide your own JSON waypoints in `autonomy_mode:=custom`. |

**Recommended order:** validate unpowered buoyancy and motor direction → run the
three motion tests (30° yaw, 3 m forward, 0.35 m down) → run the cat reference →
create, validate, and execute a bust or wreck inspection plan → evaluate images.
The BlueBoat Grand Prix is an independent challenge and does not need to run
concurrently with the BlueROV2 scene. Do not run two publishers driving the
same thruster setpoints at the same time.

---

## 1. Overview and objectives

| Challenge | Platform | Objective | Availability |
| --- | --- | --- | --- |
| **BlueBoat Grand Prix** | BlueBoat, two differential thrusters | Navigate a buoy-defined course using PID, RL, computer vision or a hybrid controller; minimise penalty-adjusted completion time. | Implemented; verify in Stonefish |
| **Underwater Survey and Mapping** | BlueROV2 | Collect overlapping images of the cat and bust (basic challenge) or shipwreck (advanced challenge); evaluate survey geometry and optionally compare aligned COLMAP/GLOMAP points to the reference meshes. | Scenario, capture and offline evaluation provided; runtime verification required |
| **Autonomous Underwater Inspection** | BlueROV2 | Navigate around solid obstacles, select unobstructed views, and inspect targets. | Map-assisted cat reference; bust/wreck plan submission, testing and advanced perception extensions |

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
unzip -oq ~/Downloads/ROSConUK_Stonefish_Workshop_v0_8_FULL_WITH_INSTRUCTIONS.zip \
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

## 8. BlueROV2 underwater photogrammetry and 3D reconstruction

This mission uses **a dedicated underwater world**, independent of the BlueBoat Grand Prix. It contains a textured and collidable Stonefish 1.6 heightmap seabed, three static photogrammetry targets (concrete cat, marble bust and shipwreck), and a locally simulated **supplied BlueROV2 Heavy geometry and physical model** (from `bluerov2_neutrally_buoyant(1).scn` and `bluerov2_unphy.zip`), retaining the original main hull, frame, **original mass/buoyancy parameters**, all eight thrusters, and the auxiliary fixed link (`link_m`) from the supplied robot. The scene explicitly uses **1031 kg/m³ seawater** to match the original physical buoyancy configuration. For workshop compatibility the many experimental IMU variants and heavyweight extra cameras were omitted; a 960 × 720 RGB survey camera and `/bluerov2/odometry` were configured. The single IMU uses Stonefish 1.6's documented standard range/noise/yaw-drift model at 100 Hz on `/bluerov2/imu/data` (no custom IMU extensions). The original robot XML is preserved under `data/photogrammetry/bluerov2_user_original_reference.scn`. This is the supplied *unmodified geometry*, not the previous box proxy.

**Targets and recommended workflow:**

| Level | Object | Recommended survey |
| --- | --- | --- |
| Basic | Weathered concrete cat | One or two circular orbits with 60–80% overlap; vary elevation. |
| Intermediate | Marble bust | Two elevations with converging camera views; inspect feature-poor marble. |
| Advanced | Shipwreck (~22 m long) | Repeated parallel passes along both sides and above the wreck, plus oblique views for occlusions. |

The aim is to **generate your own images** by moving the simulated ROV and then reconstruct from those images. The provided meshes are *reference scene assets*, not premade survey photos. Avoid capturing a static camera view hundreds of times.

### 8.1 Launch the survey scene

Run in terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py
```

Camera image topic: `/bluerov2/camera/image_raw` (ROS `sensor_msgs/Image`). Simulated odometry is published on `/bluerov2/odometry`; the **standard Stonefish 1.6 IMU** is published on `/bluerov2/imu/data` (100 Hz, documented range, noise and yaw-drift model), alongside pressure and DVL. Experimental/modified IMU models are not used. Check what the simulator actually publishes with:

```bash
ros2 topic list | grep bluerov
ros2 topic hz /bluerov2/camera/image_raw
```

### 8.2 Move the ROV to survey the objects

**Manual control** can publish eight normalized thruster commands to `/bluerov/controller/thruster_setpoints_sim` (`std_msgs/Float64MultiArray`). Thruster order is `FrontRight, FrontLeft, BackRight, BackLeft, DiveFrontRight, DiveFrontLeft, DiveBackRight, DiveBackLeft`. Start with small setpoints and confirm directions before approaching a target.

#### Direct thruster familiarisation (without the controller)

**Terminal 1 — Start Stonefish without the waypoint controller/allocator**:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py controller:=false
```

**Terminal 2 — Test the eight thrusters individually**, using short, limited-power pulses:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop bluerov_thruster_demo --demo
```

The demo sequentially pulses each of the eight motors at normalized setpoint `+0.25`, returns all motors to zero between pulses, and stops the motors on exit. The sign of the resulting motion depends on propeller orientation and inverted-setpoint flags: observe the simulator before assuming positive means forward/upward.

To test **one thruster** or an **axis pattern**:

```bash
ros2 run roscon_stonefish_workshop bluerov_thruster_demo \
  --thruster FrontRight --value 0.25 --duration 2

ros2 run roscon_stonefish_workshop bluerov_thruster_demo \
  --axis heave --value 0.25 --duration 2

ros2 run roscon_stonefish_workshop bluerov_thruster_demo \
  --axis surge --value 0.25 --duration 2
```

Other axis patterns: `sway`, `yaw`. The demonstration uses the allocation patterns from `thruster_adapter.py` but they are **not claimed to be calibrated** for the real vehicle. Commands are limited to ±0.25 per thruster and all outputs return to zero. Stop any other publisher on `/bluerov/controller/thruster_setpoints_sim` before running the demo. These are **simulation-only** exercises and should not be used to command a real vehicle without additional safety measures.

**Terminal 3 — Observe vehicle response**:

```bash
ros2 topic echo /bluerov2/odometry
```

### BlueROV2 waypoint PID (TURN → MOVE → SETTLE, fast transit profile)

The **waypoint controller** uses a simple heading-first feedback law: rotate
until roughly aligned, drive forward toward the goal, hold the requested NED
depth, and settle to the requested final yaw. There is no automatic sideways
motion or dependence on the odometry velocity-frame convention. The controller
uses **one** 8×4 motor allocator derived from the provided Logitech joystick
script; do not run another thruster publisher at the same time. The original
neutral-buoyancy vehicle configuration remains unchanged.

**First test the controller without autonomy or obstacles**:

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  controller:=true autonomy:=false
```

In another terminal request a modest **2-metre relative forward waypoint**:

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo \
  --forward 2 --seconds 45
```

Then check yaw independently (in a fresh simulation or after stopping the
previous demo):

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo \
  --turn-deg 30 --seconds 30
```

Test a small depth change separately:

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo \
  --down 0.4 --seconds 30
```

Monitor the controller and world-NED odometry in separate terminals:

```bash
ros2 topic echo /bluerov2/waypoint_pid/status
ros2 topic echo /bluerov2/odometry
```

The debug status shows `TURN`, `MOVE`, or `SETTLE`, and distance, depth, and
heading errors. In Stonefish **positive Z is deeper**. The original joystick
inputs are not guaranteed to have the same sign convention as NED motion.
This release starts with `heave_sign: -1.0`, `yaw_sign: -1.0` based on the
reported upward motion and the uploaded joystick controls, but **the sign
must be confirmed in your Stonefish simulator**. If a commanded `--down 0.4`
motion makes `odometry.pose.pose.position.z` decrease instead of increase,
flip `heave_sign`. If a positive `--turn-deg 30` moves yaw the wrong way,
flip `yaw_sign`. Both settings are in `config/bluerov_pid.yaml`; restart
launch after modifying the file. You may also test the underlying motor
patterns with `controller:=false` and `bluerov_thruster_demo --axis heave` or
`--axis yaw` at `--value 0.25` as shown above. **Do not run those motor tests
while the PID controller is active.**

The control gains and rates are *starting values*, not a
claim of runtime stability. The mission's starting turn no longer commands
an immediate depth change, and the first SEARCH waypoint keeps launch depth.
The PID parameters are now actually loaded from `config/bluerov_pid.yaml`.

For custom waypoint navigation, publish a `geometry_msgs/PoseStamped` to
`/bluerov2/setpoint` at least once every 5 s, using the **world NED** frame.
The waypoint demo publishes at 5 Hz. The cat autonomy example publishes at
2 Hz. A stale goal or stale odometry results in zero motor commands.

The static scene assets are located at approximately `(x,y) = (-10,-6)` cat; `(-10,5)` bust; `(19,6)` shipwreck, in Stonefish NED coordinates (metres). The BlueROV2 starts near `(-16,-6)`, depth 6.1 m; bottom near depth 9.4 m. Use `data/photogrammetry_site_plan.png` for a general reference, not as an exact navigation trajectory.

#### Faster PID profile and repeatable live tests

The default PID now has **faster long-distance surge** while retaining a
slower approach in the final 2 m of each goal. The motor setpoints are
normalised commands, **not metres/second**. Current initial simulation
parameters are in `config/bluerov_pid.yaml`:

| Parameter | Default | Function |
| --- | ---: | --- |
| `max_thruster_setpoint` | 0.65 | Maximum absolute output of any motor; higher commands need sign and collision checks |
| `max_forward` | 0.58 | Surge allocation request on long transits |
| `kp_forward` | 0.28 | Forward distance feedback |
| `max_yaw` | 0.34 | Yaw correction limit |
| `turn_threshold_rad` | 0.52 | Turn-in-place when farther than about 30° from course |
| `xy_tolerance` | 0.65 m | Horizontal goal tolerance |

Do **not** increase the heave limit or remove the depth damping just to make
the vehicle faster. Because the actual thruster response depends on Stonefish
physics and your motor configuration, these gains are *untested runtime
starting values*, not guaranteed stable values.

Before autonomy, run the three physical tests in the obstacle-free scene:

**Terminal 1:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  controller:=true autonomy:=false
```

**Terminal 2 — automated three-movement test (yaw → 3 m forward → depth):**

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_test \
  --timeout 55 \
  --output ~/roscon_photogrammetry/waypoint_pid_test.json
```

The script publishes only `/bluerov2/setpoint`, while the waypoint PID is
the **only** motor publisher. It reports per-step elapsed wall time, final XY,
depth and yaw errors, and a PASS/FAIL result. A real Stonefish run is required;
unit tests cannot establish that the vehicle moved correctly. If the yaw or
depth test fails, check `yaw_sign` / `heave_sign` in
`config/bluerov_pid.yaml` before changing other gains.

Individual diagnostics remain available:

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --forward 3 --seconds 45
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --turn-deg 30 --seconds 30
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --down 0.4 --seconds 35
ros2 topic echo /bluerov2/waypoint_pid/status
```

Only one waypoint-demo/mission publisher should be active at a time.

### 8.3 Record original camera images

Run in terminal 2 while driving:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop photogrammetry_capture
```

Each image is saved in `~/roscon_photogrammetry/survey/images/`, with a `manifest.csv` including ROS timestamps and optional odometry for diagnostics. Images are saved at least 1 s apart when moving; the recorder prefers frames separated by 0.18 m and records occasional stationary views. These are **actual simulated camera images**, not images rendered from the original OBJ files offline.

An alternative launch is `ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py capture:=true` (it starts the recorder automatically). Do not run the recorder twice.

### 8.4 Run sparse 3D reconstruction

Install COLMAP on your system, capture at least 30–80 overlapping photographs of one object, and run:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
bash scripts/run_photogrammetry_colmap.sh \
    ~/roscon_photogrammetry/survey/images \
    ~/roscon_photogrammetry/colmap_result
```

The script extracts and matches image features and runs incremental COLMAP mapping. It assumes one simulated 960 × 720 camera with a 70° horizontal FOV. Camera intrinsics are an initial pinhole approximation and should be calibrated or refined. **COLMAP/photogrammetry cannot be guaranteed to succeed** for every survey, especially if the images have low overlap, weak texture, image blur or poor viewpoint diversity. GLOMAP may be substituted if installed separately. Dense reconstruction is optional. Separate acquisition and geometric evaluation scripts are provided in Section 9.

### 8.5 Seabed appearance, terrain relief and survey lighting

The seabed is a **collidable 88 × 88 m Stonefish heightmap** with restored
spatially varied relief (approximately **1.03 m total elevation range**). The
terrain geometry is in `data/photogrammetry/terrain_height.png`, a 16-bit,
512 × 512 grayscale file. The survey target locations use the original flat
pads, so the cat, bust, and shipwreck placements are unchanged. The terrain
heightmap is **synthetic**, not depth information inferred from the colour image.

The colour map is the **exact uploaded `sand.jpg`**, saved at
`data/photogrammetry/textures/seabed.jpg` **without modifications**. The scene
uses a more frequent Stonefish terrain mapping, `uv_scale="1.0"` rather than
the old `0.12`, to reduce the apparent sand-grain size on the 88 m floor.
The sand JPG itself is **not resized or edited**. There is **no generated mosaic texture, colour-patch overlay,
or synthetic rock geometry**.

Six fixed scene lights (three near the survey targets and three broader seabed
fill lights) can appear as **white spheres** in the visualisation. They illuminate
the survey environment; they are not targets, buoys, physical obstacles, or
components of the vehicle.

The original BlueROV2 Heavy retains its geometry, eight thrusters, user-supplied
ballast/neutral-buoyancy configuration, and the **standard 100 Hz Stonefish
IMU**. The two BlueROV2-mounted survey lamps have been **removed**. Static
survey-site lighting is retained for observing the photo targets underwater.

Stonefish 1.6 supports `uv_scale` on a terrain's `<look>` XML element. To tune
the apparent sand grain/feature size, edit
`data/scenarios/bluerov2_photogrammetry.scn` and adjust:

```xml
<static name="Seabed" type="terrain">
  <height_map filename="photogrammetry/terrain_height.png"/>
  <dimensions scalex="0.171875" scaley="0.171875" height="1.2"/>
  <material name="Rock"/>
  <look name="Seabed" uv_scale="1.0"/>
  <world_transform xyz="0 0 10.0" rpy="0 0 0"/>
</static>
```

The separate `scripts/generate_photogrammetry_terrain.py` can generate
additional synthetic heightmaps or load a known grayscale depth map using
`--height-source`. Regenerating changes the original terrain geometry; the
packaged heightmap above is the recommended default for this workshop.

When upgrading an older installation, remove the now-unused
`data/photogrammetry/rocks/` folder. The current scene contains no rock bodies,
even if that unused folder remains on disk. Graphical verification in the
local Stonefish installation remains necessary.

### 8.6 Submission suggestions

An image dataset, camera calibration, registered COLMAP/GLOMAP poses, a sparse point cloud and a brief explanation of survey strategy form a reproducible outcome. For the advanced wreck, include additional oblique/side observations. Geometry completeness and registration quality are more meaningful than the raw number of photos taken.

### BlueROV2 IMU and physical model

The survey vehicle is derived from the supplied `bluerov2_neutrally_buoyant(1).scn` and `bluerov2_unphy.zip`, not a box-shaped proxy. Its original eight thrusters and physical components are kept. One standard Stonefish 1.6 IMU sensor is used, based on the [official IMU example](https://stonefish.readthedocs.io/en/latest/sensors.html#imu):

```xml
<sensor name="IMU" rate="100.0" type="imu">
  <range angular_velocity="10.0 10.0 5.0" linear_acceleration="10.0"/>
  <noise angle="0.1 0.1 0.5" angular_velocity="0.05" yaw_drift="0.001" linear_acceleration="0.1"/>
  <history samples="1"/>
  <origin xyz="0.1 0.0 0.0" rpy="0.0 0.0 0.0"/>
  <link name="base_link"/>
  <ros_publisher topic="/bluerov2/imu/data"/>
</sensor>
```

```bash
ros2 topic echo /bluerov2/imu/data --once
ros2 topic echo /bluerov2/dvl --once
ros2 topic echo /bluerov2/pressure --once
```

A standard IMU is **not** a navigation ground-truth source. `/bluerov2/odometry` remains a simulation baseline for the introductory waypoint controller. Observe measured sensor outputs when developing a realistic navigation algorithm. All original masses, volumes, buoyancy flags, ballast positions, shell thicknesses and thruster geometry from the supplied `.scn` are preserved. The auxiliary fixed link from the original scene has also been restored. The ocean uses an explicit density of **1031 kg/m³**, matching the intended nearly neutral configuration. A static force-balance check (buoyant-box volumes + HeavyFit shell area) predicts an error below 0.01 kg, but this is **not a Stonefish dynamic proof**: verify by starting with `controller:=false` and no thruster commands, then observe depth drift for 60–120 simulated seconds. If it drifts, record its direction and magnitude before adjusting any ballast.


## 9. Survey evaluation and inspection autonomy

The underwater work contains two **independently evaluated photogrammetry challenges**
and an optional third, autonomous inspection challenge. These evaluations do **not**
change the BlueBoat Grand Prix scoring.

### 9.1 Photogrammetry evaluation: cat + marble bust

Collect images while moving around **both** objects and vary camera height.
The default recorder saves PNGs and a `manifest.csv` under
`~/roscon_photogrammetry/survey/`. Separate capture folders are recommended
for each team and each attempt. Once images have been recorded, run:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge statues
```

The report includes individual scores for the concrete cat and marble bust
plus their equally weighted average (0–100). For each object, the acquisition
proxy awards **25 points for photo count, 60 for azimuthal viewpoint diversity**
(12 sectors around the object), and **15 for two camera height bands**. It only
counts images that exist on disk and have valid simulated odometry. A target
view is eligible roughly 1.6–5.5 m away from that object's nominal site.

This proxy does **not** verify that the target actually appears in the camera
frame, that the pixels are sharp, or that COLMAP reconstructed the object.
The aim is to provide fast, deterministic **survey-planning feedback**.

### 9.2 Photogrammetry evaluation: shipwreck

The shipwreck is much larger (~18 m long); use passes along **both sides and
above** the hull, at different depths. Run:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge wreck
```

The shipwreck report has a **separate score out of 100**. The image-count
component is 25 points, **18 spatial viewpoint cells** (three lanes × six
longitudinal sections) contribute 60 points, and two depths contribute 15.
The combined command `--challenge both` writes both scores to
`acquisition_scores.json`. This is a **viewpoint-sampling proxy**, not a
geometric accuracy score; the source meshes remain the reference.

### 9.3 Optional reconstruction geometry assessment

Reconstruct each target with COLMAP/GLOMAP and obtain a reconstructed 3D point
cloud. **COLMAP's monocular output has unknown metric scale and arbitrary
coordinates.** Before comparing it against the Stonefish reference mesh,
register the cloud to the correct site in **world NED metres** and crop the
cloud to the selected object. That registration must be supplied explicitly;
the evaluator deliberately does not assume that raw COLMAP coordinates are
metric or aligned.

Install the optional dependencies in the reconstruction environment:

```bash
python3 -m pip install scipy trimesh
```

For an aligned PLY point cloud, run (repeat for `bust` or `wreck`):

```bash
ros2 run roscon_stonefish_workshop photogrammetry_geometry_eval \
  --target cat --cloud /path/to/aligned_cat.ply \
  --reference-dir ~/ros2_ws/src/roscon_stonefish_workshop/data/photogrammetry
```

The geometric evaluator reports nearest-neighbour **precision, completeness,
and F-score** against sampled reference geometry at a distance threshold of
0.10 m (statues) or 0.25 m (wreck). The shipwreck reference combines all available
`wr_*.obj` parts. This is an optional diagnostic, **not** part of the
acquisition-point score.

The package can estimate the similarity transform from the COLMAP camera
centres and `manifest.csv` (for **evaluation only**) if COLMAP registered at
least six of the recorded frames. Convert the sparse model to text first:

```bash
mkdir -p ~/roscon_photogrammetry/colmap_result/sparse_txt
colmap model_converter \
  --input_path ~/roscon_photogrammetry/colmap_result/sparse/0 \
  --output_path ~/roscon_photogrammetry/colmap_result/sparse_txt \
  --output_type TXT
ros2 run roscon_stonefish_workshop photogrammetry_align \
  --manifest ~/roscon_photogrammetry/survey/manifest.csv \
  --colmap-images ~/roscon_photogrammetry/colmap_result/sparse_txt/images.txt \
  --output ~/roscon_photogrammetry/colmap_to_world.json
```

Create the PLY cloud from the same sparse model:

```bash
colmap model_converter \
  --input_path ~/roscon_photogrammetry/colmap_result/sparse/0 \
  --output_path ~/roscon_photogrammetry/colmap_result/sparse_points.ply \
  --output_type PLY
ros2 run roscon_stonefish_workshop photogrammetry_geometry_eval \
  --target cat \
  --cloud ~/roscon_photogrammetry/colmap_result/sparse_points.ply \
  --alignment ~/roscon_photogrammetry/colmap_to_world.json --auto-crop
```

Repeat the last command using `--target bust` or `--target wreck`.
**Keep only images belonging to the same reconstruction when fitting the
transform.** Registration residuals are written next to the JSON transform.
Inspect a cloud overlay before interpreting a geometric score. Camera
odometry is used by this evaluation tool only, never as an input to COLMAP.

A 4×4 JSON transform (reconstruction frame to world NED metres) may be
provided with `--alignment /path/to/transform.json`. It must include **correct
scale**, not just camera translation/rotation. Cloud-to-reference alignment is
required for meaningful distances.

### 9.4 Challenge 3: autonomous underwater inspection with obstacles and occlusion

Challenge 3 uses the same BlueROV2, camera, seabed and three photogrammetry
objects as Challenge 2. Select the separate obstacle-rich scenario:
`bluerov2_autonomy_obstacles.scn`. It adds **11 solid, collidable stone blocks
and pillars** positioned near the cat, bust, shipwreck and the transit area.
The barriers can block both the vehicle's route and camera visibility.

**Two levels of source code are included:**

- **CAT — working map-assisted reference:** autonomous search waypoints,
  collision-checked A* routes on a known obstacle map, occlusion-aware
  candidate camera positions at two depths, automated position/yaw control,
  image capture and return-to-home. The algorithms are in
  `autonomy_geometry.py` and `inspection_reference.py`. This is the ONLY
  complete reference. The planning map contains known static obstacle and
  target coordinates. It is *not* semantic target recognition, online mapping,
  reconstruction-aware next-best-view planning, or guarantee of 3D clearance.
- **MARBLE BUST and SHIPWRECK — incomplete exercise starters:** safe initial
  **search waypoints only**, followed by an explicit `STARTER_READY` status.
  Search, image-based target identification, online obstacle mapping,
  collision-checked approach, viewpoint selection based on observed geometry,
  inspection and return must be implemented. These extension points appear in
  `roscon_stonefish_workshop/autonomy_exercises.py`. The scene provides the
  geometry and ROS camera/odometry interfaces, **not** a solved route.

#### Cat: run the reference solution

Terminal 1 (from any workspace terminal with ROS and this package sourced):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=cat autonomy_mode:=reference
```

The controller uses the user-supplied 8×4 Logitech thruster allocation,
with one direct 8-motor publisher and no competing adapter. The
robot publishes `nav_msgs/Odometry` on `/bluerov2/odometry`; the PID consumes
that topic and publishes `Float64MultiArray` commands to
`/bluerov/controller/thruster_setpoints_sim`. **Do not run the direct-thruster
demo or `thruster_adapter` simultaneously with `bluerov_pid`**.

Check the mission and the camera:

```bash
ros2 topic echo /bluerov2/inspection/status
ros2 topic hz /bluerov2/camera/image_raw
ros2 topic echo /bluerov2/odometry --once
```

The reference autonomously sends search, approach, inspection and return goals;
its approximate occlusion-aware viewpoint selection uses the *known* static
obstacles. It does **not** look for the object in the image to discover its
location. Its `visual_required` switch is an optional integration hook for a
separate detector (not supplied as a functioning vision model). Using
`visual_required:=true` without publishing detections will prevent completion.

#### Bust and wreck: start with search, then develop and evaluate a custom plan

```bash
# Marble bust — starts a safe search, then waits for algorithm implementation.
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=bust autonomy_mode:=starter

# Shipwreck (run separately; stop the previous instance first).
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=wreck autonomy_mode:=starter
```

The starter deliberately stops after its search prefix. A complete custom-plan
execution path is provided below; advanced camera-driven autonomy can replace
it. To implement full perception-driven autonomy, complete the functions in
`autonomy_exercises.py`:
`search_waypoints`, `discover_target`, `update_obstacles`, `plan_safe_route`,
`choose_next_best_views`, and `should_return_home`. The inputs should come
from permitted camera, IMU, DVL, depth and, if implemented, sonar observations;
the scene's nominal obstacle/target positions are useful for debugging and
comparison, but the advanced target discovery task should use visual evidence.

**Reference guard:** launching `target:=bust autonomy_mode:=reference` or
`target:=wreck autonomy_mode:=reference` raises a clear error. The complete
reference is restricted to the cat.

#### Bust and shipwreck: practical route from starter to completed mission

The goal of Challenge 3 is to select a safe route, capture multiple useful
camera views and return. The **cat** is the fully authored worked reference.
For the other two targets there are **two explicit levels**:

**Level 1 — map-assisted inspection, attainable during the workshop.**
Use the known static obstacle map for transit navigation and focus on
choosing safe, non-occluded inspection viewpoints. The participant chooses
the path goals; this package supplies generic waypoint execution, A* transit
between those goals, and evaluation. A successful map-assisted mission is not
proof of visual target discovery.

**Level 2 — perception-driven autonomy (advanced).** Implement a detector
using `/bluerov2/camera/image_raw` and camera calibration, use sensor-derived
local obstacle representations instead of the published static map, and
propose next-best views from observed target geometry. Functions to replace
are in `roscon_stonefish_workshop/autonomy_exercises.py`.

To run Level 1 for the bust:

**1. Create a custom plan** from the intentionally empty template:

```bash
cp ~/ros2_ws/src/roscon_stonefish_workshop/data/autonomy/bust_plan_TEMPLATE.json \
  ~/bust_plan.json
```

Edit `~/bust_plan.json` and replace the empty `waypoints` array with your own
NED world coordinates. Each entry is a camera/vehicle pose, with yaw in
**radians**; for instance, the **schema only** is:

```json
{
  "target": "bust",
  "frame": "world_ned",
  "waypoints": [
    {"x": -18.0, "y": 3.0, "z": 6.5, "yaw": 0.0, "phase": "SEARCH"},
    {"x": -17.0, "y": 4.0, "z": 7.0, "yaw": 0.0, "phase": "APPROACH"}
  ]
}
```

**These two sample poses are not a completed route.** Add at least **four**
`INSPECT` viewpoints for the bust, from at least three different azimuth
sectors. The wreck requires **six** or more viewpoints and several distinct
view directions and heights; a single overhead pass rarely covers its sides.
For each target, avoid collidable structures and consider target occlusion.
The validator uses a conservative **known-map/2D** geometry proxy, not a
real camera renderer.

**2. Check the plan before simulation:**

```bash
ros2 run roscon_stonefish_workshop autonomy_plan_check \
  --target bust --plan ~/bust_plan.json
```

Invalid poses, missing stages, blocked inspection views and insufficient
viewpoint diversity are reported as errors. Look at `data/scenarios/
bluerov2_autonomy_obstacles.scn` and the cat example
`autonomy_geometry.py` to understand the safe operating area.

**3. Execute the custom route using the same PID and mission logger:**

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=bust autonomy_mode:=custom plan_file:=$HOME/bust_plan.json
```

The generic runner inserts collision-aware **transit** segments using the
known static obstacle map and automatically adds **RETURN** to the launch
position. It does not create the target's inspection viewpoints. Review the
recorded route and photos and revise the JSON until the mission succeeds.

For the shipwreck, follow the same workflow with
`data/autonomy/wreck_plan_TEMPLATE.json`, `--target wreck`,
`target:=wreck`, `~/wreck_plan.json`. Several inspection locations should
observe the hull sides and partially occluded structures at different depths.

**4. Evaluate the mission and reconstruction:**

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge both
ros2 run roscon_stonefish_workshop inspection_evaluate \
  --mission ~/roscon_photogrammetry/missions/inspection_bust_YYYYMMDD_HHMMSS.json
```

Substitute `inspection_wreck_...json` for the shipwreck. Add the
`--acquisition` argument if an image-acquisition report is available.

**5. Optional real visual detection:** Publish a JSON detection message
on `/bluerov2/autonomy/detections` as `std_msgs/msg/String`, with fields
`target`, `detected` (boolean) and `confidence` (numeric 0–1); use a
confidence above 0.65 from a genuine camera-based detector. The mission
node exposes a `visual_required` parameter that can gate progression beyond
SEARCH if a detector is running. A fake detection message is not a
substitute for perception or a valid challenge submission. The advanced
component hooks are deliberately unimplemented and may be replaced by ROS
nodes publishing candidate waypoints as `/bluerov2/setpoint`.

**Important distinction:** `autonomy_mode:=starter` remains a **search-only
exercise** with `STARTER_READY`. `autonomy_mode:=custom` actually executes a
participant-created route, while `autonomy_mode:=reference` remains
**cat-only**. The static obstacle map is a prior used by the runner; the
advanced Level 2 challenge must use onboard sensing to replace it.

#### Save and evaluate autonomy runs

Mission JSONs are saved under `~/roscon_photogrammetry/missions/` and camera
images under `~/roscon_photogrammetry/survey/images/`:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge both
ros2 run roscon_stonefish_workshop inspection_evaluate \
  --mission ~/roscon_photogrammetry/missions/inspection_cat_YYYYMMDD_HHMMSS.json \
  --acquisition ~/roscon_photogrammetry/survey/acquisition_scores.json
```

The mission report includes reached waypoints, inspection viewpoints, odometry
trajectory, return status and a *proxy* obstacle-clearance assessment based on
the published map (a 20-point penalty if any sampled path enters a safety
keep-out). The score allocates 15 approach, 35 inspection, 25 return, 10
completion efficiency and 15 optional image-acquisition points, minus the
20-point obstacle proxy penalty when applicable. The report also includes a
2D occlusion/novel-view proxy. **These are not true simulated collision-event
measurements or measured 3D surface completeness.** For photogrammetry quality,
use the separate geometric evaluator from section 9.3.

**Runtime limitation:** The complete cat reference is an implemented ROS node,
but its closed-loop trajectory following, collision avoidance and final camera
viewpoints have **not** been validated in the graphical Stonefish installation.
The new waypoint controller is deliberately simple and the motor signs remain
simulator-dependent. Before running the obstacle mission, test a forward goal,
a +30° yaw goal, and a +0.4 m depth goal with `bluerov_waypoint_demo` in the
unobstructed scene. Use `/bluerov2/waypoint_pid/status` to identify incorrect
signs or a depth-hold issue. The cat SEARCH first waypoint now holds the
vehicle's initial depth and the waypoint timeout has been extended to 150 s.

## 10. Troubleshooting and verification

### `stonefish_ros2` build error on ROS 2 Humble (`ROS2Interface.cpp:697`)

When building the upstream `stonefish_ros2` package on **Ubuntu 22.04 / ROS 2
Humble** with **GCC 11.4.0**, compilation may fail in
`src/stonefish_ros2/ROS2Interface.cpp`. The relevant expression tries to add an
`rclcpp::Duration` directly to `msg.header.stamp`, which is a ROS message
`builtin_interfaces::msg::Time` rather than an `rclcpp::Time` object.

**Example compiler error** (shown with the original source line):

```text
/ros2_ws/src/stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:697:45:
error: no match for ‘operator+’
(operand types are ‘std_msgs::msg::Header_<std::allocator<void> >::_stamp_type’
 {aka ‘builtin_interfaces::msg::Time_<std::allocator<void> >’}
 and ‘rclcpp::Duration’)
  697 | msg.events[i].ts = msg.header.stamp + rclcpp::Duration(0, abs(data[i*2+1]));
      |                    ~~~~~~~~~~~~~~~~ ^ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
```

**Cause:** `builtin_interfaces::msg::Time` is a message type, whereas
`rclcpp::Time` supports arithmetic with `rclcpp::Duration`. Convert the
header timestamp before adding the duration.

**Original expression (`ROS2Interface.cpp`, around line 697):**

```cpp
msg.events[i].ts = msg.header.stamp + rclcpp::Duration(0, abs(data[i*2+1]));
```

**Corrected expression:**

```cpp
msg.events[i].ts = rclcpp::Time(msg.header.stamp) + rclcpp::Duration(0, abs(data[i*2+1]));
```

Apply the change to the **upstream `stonefish_ros2` source file** (not to this
workshop package). If similar compiler errors appear elsewhere, use the same
explicit conversion when operating on ROS message timestamps.

Then rebuild and re-source the ROS 2 workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stonefish_ros2
source install/setup.bash
```

This one-line source change was used successfully to compile the affected
package on ROS 2 Humble. It is a **local compatibility fix**, not a claim that
all versions of `stonefish_ros2` require it; check the upstream source for an
existing fix before editing.

| Symptom | Check |
| --- | --- |
| Stonefish aborts with `Material file not found` | Ensure the corrected `opti_blueboat` OBJ assets are installed; older propeller meshes referenced missing `.mtl` files. |
| Forward/reverse motion does not match commands | Test small direct setpoints and inspect the 180° propeller pitch and inversion settings. |
| Direct BlueBoat thruster demo has no effect | Launch with `thruster_adapter:=false`; the adapter otherwise competes for the setpoint topic. |
| BlueROV2 waypoint PID does not approach goal or initially rises | Start in the obstacle-free scene. Check `ros2 topic echo /bluerov2/waypoint_pid/status`, and verify `yaw_sign` and `heave_sign` in `config/bluerov_pid.yaml`. Positive odometry Z is down. Do not run a separate `thruster_adapter`. |
| Cat reference fails in obstacle world | Verify initial vehicle pose, obstacle clearance, and that the controller follows the map-assisted waypoints. Check the mission status and report. |
| Bust or wreck reference refuses to start | Intentional: use `autonomy_mode:=starter` and implement the remaining challenge stages. |
| `/blueboat/cmd_vel` has no effect | Launch with the adapter enabled (the default), publish one Twist controller at a time and check for topic activity. |
| `race_start` cannot arm | Confirm race status, odometry, initial spawn proximity (within 1.5 m) and remaining attempt slots. |
| START crossing is not scored | Check for an **ARMED** state; crossing START during practice is intentionally ignored. |
| A run fails to finish | Verify all 12 checkpoints were crossed in order and within their gate widths, before FINISH. |
| `race_export` finds no results | Check the team identifier and confirm an official attempt was armed previously. |
| ArUco signs appear incorrectly cropped | Use the centered-texture original-size box-panel textures included with this release. |
| A simulator build succeeds but runtime fails | Inspect simulator output and run Stonefish locally; the offline tests cannot validate GPU rendering or physics. |

The simulation can run at different wall-clock speeds depending on the machine; the race measures simulation time. Comparable results still require the same physics, course and judge configuration.

## 11. Package structure and sharing

```text
roscon_stonefish_workshop/
├── README.md                           # This guide (the only README)
├── package.xml / setup.py / setup.cfg
├── launch/
│   ├── blueboat_workshop.launch.py
│   ├── bluerov2_workshop.launch.py
│   └── bluerov_controller.launch.py
├── config/
│   ├── blueboat_race.json              # Local race geometry and rules
│   └── bluerov_pid.yaml                # Fast waypoint PID gains and motor signs
├── data/
│   ├── scenarios/                      # BlueBoat Grand Prix and both ROV scenes
│   ├── robots/                          # Original simulation robot descriptions
│   ├── autonomy/                        # Bust/wreck incomplete plan templates
│   ├── photogrammetry/                  # Terrain, camera targets and textures
│   ├── bluerov2_unphy/                  # Supplied BlueROV2 meshes
│   └── opti_blueboat/                   # Supplied BlueBoat meshes
├── roscon_stonefish_workshop/
│   ├── blueboat_pid.py                  # BlueBoat controller starter
│   ├── blueboat_race_monitor.py         # Local three-attempt referee
│   ├── bluerov_thruster_demo.py         # Direct eight-thruster tests
│   ├── bluerov_pid.py                   # TURN/MOVE/SETTLE controller
│   ├── bluerov_waypoint_test.py         # Live yaw/forward/depth tests
│   ├── autonomy_plan.py                 # Custom-route validator
│   ├── autonomy_exercises.py            # Incomplete perception/NBV hooks
│   ├── inspection_reference.py          # Cat example and custom-plan runner
│   └── survey_evaluation.py             # Statue/wreck acquisition metrics
├── scripts/
│   ├── run_photogrammetry_colmap.sh
│   └── show_blueboat_leaderboard.py
└── tests/                               # Repeatable offline validation
```

The photogrammetry scene and the more difficult obstacle-and-occlusion scene
are separate launch options. The cat is the only solved autonomous inspection
reference. Both the bust and shipwreck require custom search/inspection
viewpoints; the included mission runner handles their submitted waypoints
but does not discover those viewpoints for the user.

The public workshop package is intended to be used from its source repository.
Please link to that repository rather than mirroring this bundle. This request
cannot override the Apache-2.0 software licence or rights granted by any
third-party asset licence. The overhead BlueBoat map and waypoint CSV are
released separately at the announced event time.

### Photogrammetry texture and lighting troubleshooting

The supplied visual OBJ meshes contain explicit UV coordinates **and surface normals**.
Stonefish applies the diffuse textures using the scenario `<look texture=...>` entries;
OBJ `.mtl` references are intentionally not needed. Fixed, broad survey-site fill lights support sufficiently visible images at
underwater depth. The fill lights are intended for the introductory workshop and may be
removed for an advanced low-light survey. To diagnose missing assets, examine the
Stonefish console for texture or model loading errors and verify the launch argument
`simulation_data` points to this package's `data` directory. If an object remains
black, first verify `data/photogrammetry/textures/*.jpg` exists in the installed
package and that the visual OBJ has `vn` records and `f v/vt/vn` faces.

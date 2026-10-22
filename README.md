# ROSCon UK 2026 · Underwater Robotics with ROS and Stonefish

**A hands-on marine robotics workshop** · ROS 2 Humble · Stonefish 1.6  
**BlueBoat Grand Prix** · **BlueROV2 Photogrammetry** · **Autonomous Inspection**

> **Start here:** The BlueBoat race starts in **unlimited practice mode**. You receive **three official attempts per team and course**, and an attempt is spent the moment you successfully **arm** it—not when you cross the start gate. The official race must use live sensor observations; navigation from hidden course files is a **FAIR PLAY VIOLATION**. The local judge remains active.

This repository contains simulation environments, ROS 2 controllers, sensor interfaces, examples with `TODO` exercises, and local evaluation tools. You run everything on your **own computer**: no central ROS network or common `ROS_DOMAIN_ID` is needed.

| Challenge | Vehicle | What you will build |
|:--|:--|:--|
| **01 · BlueBoat Grand Prix** | Differential-thrust surface vessel | Perception-based control to follow a marked race course, pass gates and finish with the best penalty-adjusted time. |
| **02 · Underwater Survey & 3D Mapping** | BlueROV2 Heavy | Collect overlapping simulated RGB images of a cat, marble bust or shipwreck and reconstruct/evaluate geometry. |
| **03 · Autonomous Underwater Inspection** | BlueROV2 Heavy | Plan safe inspection viewpoints around obstacles and occlusions; extend the incomplete perception and planning examples. |

**Scope:** The supplied BlueBoat PID and camera examples are *starters*, not race solutions. The fully implemented, **map-assisted** BlueROV2 inspection reference is **cat-only**; bust and wreck include incomplete starters and an executable custom-plan path. Always verify vehicle motion, cameras and rendering in your own Stonefish installation.

---

## Contents

1. [Workshop essentials](#1-workshop-essentials)
2. [Install and build](#2-install-and-build)
3. [BlueBoat Grand Prix: quick start](#3-blueboat-grand-prix-quick-start)
4. [Race rules: practice, three attempts and scoring](#4-race-rules-practice-three-attempts-and-scoring)
5. [Fair Play Violation: what is and is not allowed](#5-fair-play-violation-what-is-and-is-not-allowed)
6. [BlueBoat controllers: direct motors, PID and vision](#6-blueboat-controllers-direct-motors-pid-and-vision)
7. [Submitting results and local leaderboard](#7-submitting-results-and-local-leaderboard)
8. [BlueROV2: photogrammetry and reconstruction](#8-bluerov2-photogrammetry-and-reconstruction)
9. [BlueROV2: autonomous inspection](#9-bluerov2-autonomous-inspection)
10. [Troubleshooting](#10-troubleshooting)
11. [Project layout, limitations and licences](#11-project-layout-limitations-and-licences)

---

## 1. Workshop essentials

### Requirements

- **Ubuntu 22.04**, **ROS 2 Humble**, Python 3, **Stonefish 1.6** and `stonefish_ros2`.
- A GPU/OpenGL environment capable of running Stonefish (OpenGL **4.3** or later).
- ROS packages used here include `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `rclpy`, `ament_index_python` and `cv_bridge` for camera examples.
- **Optional:** OpenCV with `cv2.aruco` for ArUco detection; COLMAP/GLOMAP and Python geometry packages for 3D reconstruction/evaluation.
- **COLA2 is not required** for the BlueBoat Grand Prix.

Helpful references: [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) · [Stonefish documentation](https://stonefish.readthedocs.io/en/latest/) · [stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2).

### The four rules everyone should know

1. **Practise as often as you want.** Practice runs never reduce your official-attempt balance.
2. **You have three official attempts.** Successful arming consumes one, even if you cancel, crash, fail to finish or restart Stonefish.
3. **Complete the gates in order.** Cross green **START → CP01 … CP12 → red FINISH**. The best *finished* penalty-adjusted time wins.
4. **Navigate from live sensors, not hidden maps.** The race generator and local judge may read the organiser course configuration; participant navigation must not.

---

## 2. Install and build

Start with a working `~/ros2_ws` in which `stonefish_ros2` already builds and launches. Install useful build tools as needed:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep unzip
```

Place the package directory at `~/ros2_ws/src/roscon_stonefish_workshop`. If installing from a distributed ZIP (instead of cloning the repository), extract it and copy the **package directory itself**:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

# If replacing an old copy, move it outside src/ first.
if [ -d src/roscon_stonefish_workshop ]; then
  mv src/roscon_stonefish_workshop \
    "$HOME/roscon_stonefish_workshop_backup_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p /tmp/roscon_stonefish_release
unzip -q /path/to/your/workshop-release.zip -d /tmp/roscon_stonefish_release
cp -a /tmp/roscon_stonefish_release/rosconUK_workshop_stonefish \
  src/roscon_stonefish_workshop

colcon build --symlink-install --packages-select roscon_stonefish_workshop
source install/setup.bash
ros2 pkg prefix roscon_stonefish_workshop
ros2 pkg prefix stonefish_ros2
```

> If your download is already an unpacked repository named `roscon_stonefish_workshop`, copy that directory directly instead. The ZIP directory name above matches the supplied workshop bundle. **Source `/opt/ros/humble/setup.bash` and `~/ros2_ws/install/setup.bash` in every new terminal.**

Optional offline checks (these do **not** validate GPU rendering, propeller directions or closed-loop flight dynamics):

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 -m unittest discover -s tests -v
```

---

## 3. BlueBoat Grand Prix: quick start

The **Chiocciola v4** course is an approximately **225.8 m**, non-looping track with yellow boundary buoys, 12 checkpoints, paired ArUco direction signs, a **green START** and a **red FINISH**. It includes an opening straight, S-bends, a tightening spiral, a short straight, a 90° elbow and a finishing straight. You do **not** receive a waypoint solution.

### Terminal A · Launch the race (practice by default)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py \
  team:=Team_01
```

The simulator launches with the local judge and the high-level thrust adapter enabled. Look for a message similar to:

```text
BlueBoat judge: team=Team_01, PRACTICE mode; 3/3 official attempts remain
```

### Terminal B · Inspect the camera without moving

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run roscon_stonefish_workshop blueboat_camera_steering
```

The camera starter **does not move the boat by default** (`enable_motion:=false`). It reports tentative yellow targets and visible ArUco IDs.

When you have confirmed the RGB topic and the steering sign visually, stop that node and try **low-speed practice**:

```bash
ros2 run roscon_stonefish_workshop blueboat_camera_steering \
  --ros-args -p enable_motion:=true -p surge_speed:=0.15
```

This is **not** a completed course follower. It steers towards the midpoint of the two largest yellow image blobs, which might not be the correct gate. Finish the `TODO` tasks in Section 6 before using it for official racing.

### Terminal C · Observe the judge

```bash
ros2 topic echo /blueboat/race/status
```

The message is a `std_msgs/msg/String` containing JSON. During practice it reports `"state":"practice"` and the remaining official attempts. The public status intentionally **does not publish cross-track distance or hidden centreline position**.

To verify the sensors:

```bash
ros2 topic list | grep blueboat
ros2 topic hz /blueboat/camera/image_raw/image_color
ros2 topic info -v /blueboat/camera/image_raw/image_color
ros2 service list | grep /blueboat/race
```

> **One active controller at a time.** Do not run the heading PID and camera steering together, and do not run a direct-thruster demo while the thrust adapter is publishing to the motor topic.

---

## 4. Race rules: practice, three attempts and scoring

### 4.1 Practice is unlimited

The judge **always starts in Practice Mode**. You can test algorithms, drive the whole circuit, restart the simulator and experiment with controller gains without consuming an official attempt. Practice runs are **not** added to the official leaderboard. Crossing START during practice does not start official timing.

**Practice is not a fair-play exception:** hidden-course/waypoint navigation is forbidden in both practice and official racing.

### 4.2 Three official attempts: exactly when do they count?

| Action | Consumes an official attempt? | What happens |
|:--|:--:|:--|
| Launch Stonefish in practice | **No** | Unlimited practice begins. |
| Cross START and checkpoints in practice | **No** | No official time is recorded. |
| Try to arm without odometry, near-spawn position or available slots | **No** | Judge refuses the request. |
| **Successfully arm** with `race_start` | **YES — immediately** | One of three slots is reserved and a persistent `report.json` is created. |
| Wait after arming, before crossing START | **Already consumed** | Timing has not started, but the slot cannot be refunded. |
| Cross green START after arming | **Already consumed** | The START-to-FINISH clock begins. |
| Cancel/disarm an armed run | **YES** | Saved as `DNF_CANCELLED`. |
| Timeout, leave unfinished or restart after arming | **YES** | A DNF or unfinished reserved record still counts. |
| Finish a valid run | **YES** | An adjusted time becomes eligible for ranking. |

The judge allows **at most three successfully armed attempts for each team identifier + course hash** in the configured local results directory. An unchanged installation preserves this history across restarts. The team identifier must remain the same for all three runs. **Do not rename your team or delete results to request extra chances.**

### 4.3 How to run one official attempt

**Before arming:** finish practice; restart the scene to return to the start; launch **only one** controller; verify odometry is available. The boat must be within **1.5 m of its spawn position**.

**Terminal A — simulator:**

```bash
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py \
  team:=Team_01
```

**Terminal B — prepare your sensor-based controller safely:**

```bash
# Example: basic PID, NOT a solved race controller.
# Keep forward speed at ZERO until arming succeeds.
ros2 run roscon_stonefish_workshop blueboat_pid --ros-args \
  -p surge_command:=0.0 -p target_heading_rad:=0.0
```

**Terminal C — arm when you are genuinely ready to spend one attempt:**

```bash
ros2 run roscon_stonefish_workshop race_start
```

Equivalent ROS service:

```bash
ros2 service call /blueboat/race/arm \
  std_srvs/srv/SetBool "{data: true}"
```

Expected successful acknowledgement:

```text
Official attempt 1/3 ARMED. Cross the green START gate forward to begin timing.
```

Only **after successful arming**, enable motion in your controller. For the stationary PID example above, this can be done from a separate terminal:

```bash
ros2 param set /blueboat_pid surge_command 0.15
```

For a custom controller, enable its motion in the equivalent safe way. The public JSON state is `waiting_for_start` while armed but not yet started; it becomes `running` after the forward START crossing. **The attempt was already deducted at arming.**

The required order is:

```text
PRACTICE
   └─ race_start (successful) → ARMED / waiting_for_start  [slot consumed]
                                    │
                                    └─ forward START crossing → RUNNING
                                                │
                                                ├─ CP01 → CP02 → ... → CP12
                                                │
                                                └─ forward FINISH → FINISHED
```

The judge checks **ordered gate crossings** from the simulator's position data. Your controller does not need to publish ArUco detections for an official checkpoint to count. The next expected gate is checked; crossing a later gate first **does not** skip earlier ones.

To cancel an active attempt (it **still counts**):

```bash
ros2 run roscon_stonefish_workshop race_start --disarm
```

For attempts 2 and 3, stop the controller and simulator, restart at the initial pose **using the same team name**, and arm again. The results directory must not be cleared. When `attempts_remaining` reaches zero, further arming is rejected.

### 4.4 Timing, penalties and ranking

A finished run's result is its **elapsed race time plus penalties**:

```text
Final time (s) = START-to-FINISH time (s)
               + 8 × number of off-track excursions
               + 2 × total seconds spent outside the corridor
```

**All penalty terms are added**, so a longer time or more off-track violations can never improve the score. This plain-text formula is intentional: it renders consistently on GitHub and in Markdown viewers that do not support LaTeX.

- **`T_START→FINISH`**: time between the correct START and FINISH crossings, in **simulation seconds** (not wall-clock seconds).
- **`N_offtrack`**: number of separate departures from the permitted course corridor after timing begins. Each adds **8 seconds**.
- **`T_outside`**: accumulated number of simulation seconds outside that corridor during the run; each second adds **2 penalty seconds**.
- The judge uses a boundary margin and re-entry hysteresis to avoid double-counting a noisy crossing.

**Example:** 150 s elapsed, two off-track excursions, five seconds outside gives **150 + 2×8 + 2×5 = 176 s**.

A run lasting over **1,200 simulation seconds** from START without a valid FINISH is **DNF_TIMEOUT**. A DNF uses a slot but has **no ranked finish time**. The winning result is the **lowest adjusted time among the team's completed official attempts** (not the average and not simply the final attempt).

| Attempt | Outcome | Elapsed | Penalties | Ranked adjusted time |
|:--|:--|--:|--:|--:|
| 1 | FINISHED | 180 s | 12 s | 192 s |
| 2 | DNF_TIMEOUT | — | — | — |
| 3 | FINISHED | 162 s | 6 s | **168 s (best)** |

### 4.5 What does the race status mean?

```bash
ros2 topic echo /blueboat/race/status
```

Typical published keys include `state`, `next_gate`, `gates_passed`, `race_time_s`, `penalty_s`, `adjusted_time_s`, `attempts_used`, `attempts_remaining` and `official_attempt_id`.

- `practice`: no official clock and no attempt consumption.
- `waiting_for_start`: the judge is **armed**, the attempt is already consumed, waiting for the forward START crossing.
- `running`: timing and penalties are accumulating.
- `finished`: a valid completed run has been saved.
- `timed_out`: the run exceeded the time limit.

The public status is for **progress and scoring review**. The judge has private geometric diagnostics needed for scoring; these are intentionally omitted from the published status.

---

## 5. FAIR PLAY VIOLATION: what is and is not allowed

> ### FAIR PLAY — READ BEFORE WRITING YOUR CONTROLLER
>
> The BlueBoat Grand Prix is a **live-sensor navigation challenge**, not a waypoint-replay exercise. **Do not use known course coordinates to steer.** This rule applies during **practice and official attempts**.

### Allowed navigation inputs

| Allowed | Examples |
|:--|:--|
| **Live cameras** | RGB pixels, buoy detections, optical flow, segmentation and learned vision models. |
| **Observed ArUco signs** | Marker IDs actually decoded from the simulated camera image. |
| **Live navigation sensor streams** | Permitted onboard odometry, IMU, GPS/heading and vehicle motion feedback. |
| **Own controller state** | Control histories, PID memory, visual tracking and online maps estimated from allowed observations. |
| **Algorithms you create** | PID, visual servoing, RL, planning from sensor-derived geometry and hybrid methods. |

### Forbidden navigation inputs

| Forbidden | Why |
|:--|:--|
| `config/blueboat_race.json` | Contains the organiser's exact centreline/checkpoint geometry. |
| Renamed/copied route files (`.json`, `.csv`, `.yaml`, etc.) | Renaming a precomputed route does not make it an observation. |
| Generated `.scn` files parsed for buoy/gate coordinates | Extracting positions from the environment asset bypasses perception. |
| Race judge geometry, internal state or ground-truth scoring feeds | Privileged information is meant for scoring, not participant steering. |
| Reference trajectories, saved waypoint lists or replayed solution coordinates | They substitute a known route for live autonomous navigation. |
| Attempting to bypass the provided fair-play checks | A source-code workaround is not compliant competition behavior. |

**Important distinction:** The course JSON must remain in the installed package because the **organiser's scene generator and local race judge need it**. The rule restricts participant *navigation use*, not the functioning of the judge. Participants may watch the publicly documented scoring status to see race progress, but must not treat private judge geometry or saved reference trajectories as steering inputs.

### The supplied FAIR PLAY VIOLATION guard

The starter controllers `blueboat_pid` and `blueboat_camera_steering` reject user-supplied route/waypoint parameters including `course_file`, `waypoints_file`, `route_file` and `track_file`. They also reject judge-only/ground-truth topics where checked. The legacy waypoint solution deliberately refuses to run.

For instance, this prohibited parameter will cause the **participant controller** to terminate:

```bash
ros2 run roscon_stonefish_workshop blueboat_pid --ros-args \
  -p course_file:=config/blueboat_race.json
```

The error prominently states:

```text
============================================================================
FAIR PLAY VIOLATION — BLUEBOAT GRAND PRIX

Precomputed race coordinates, course JSON/CSV, generated waypoint files,
judge internals and reconstructed checkpoint positions are NOT permitted
as participant navigation inputs. Use camera, ArUco, IMU, GPS or odometry.

Reason: Participant parameter 'course_file' must not provide a precomputed route
Blocked source: config/blueboat_race.json

The LOCAL RACE JUDGE and local practice/official scoring remain enabled.
This participant controller is stopping now.
============================================================================
```

**What stops?** The **supplied participant controller that attempted the restricted access**. The BlueBoat simulation and **local judge keep running**. The guard does **not** automatically disqualify or consume an official attempt by itself: if an attempt was previously armed, its existing rules still apply.

**Technical limitation:** A `.json` file cannot execute code merely because someone opens it. This guard does not intercept arbitrary text editors, `open()` calls in custom programs, scene reverse-engineering or modified copies of the workshop. It is a **clear rule and an in-process safeguard, not tamper-proof security**. Organisers should review participant code, inspect traces and exercise independent supervision for prizes. The local three-attempt ledger likewise relies on participant-controlled files.

---

## 6. BlueBoat controllers: direct motors, PID and vision

There are **two distinct control paths**. Use only one per session:

```text
Controller → /blueboat/cmd_vel (Twist)
                ↓  [adapter ENABLED]
       blueboat_thruster_adapter
                ↓
/blueboat/controller/thruster_setpoints_sim [right, left]
                ↓
            Stonefish BlueBoat

OR: direct thruster demo → /blueboat/controller/thruster_setpoints_sim
                       [adapter DISABLED]
```

### 6.1 Direct thruster familiarisation

Disable the adapter when commanding motors directly:

```bash
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py \
  team:=Team_01 thruster_adapter:=false
```

In a second terminal:

```bash
ros2 run roscon_stonefish_workshop blueboat_thruster_demo --demo
```

Or issue one short low-power pulse:

```bash
ros2 run roscon_stonefish_workshop blueboat_thruster_demo \
  --right 0.15 --left 0.15 --duration 3
```

This workshop model uses **positive raw `[right, left]` motor commands for forward motion**, which corrects an earlier reversed-motion issue. Setpoints are **normalised commands, not m/s or Newtons**. The intro demo limits each to ±0.30, caps duration at 15 s and stops the motors on exit. **Inspect actual movement in Stonefish before increasing power.**

### 6.2 Sensor-only heading PID

Restart the simulator normally (adapter **enabled**, the default), then run:

```bash
ros2 run roscon_stonefish_workshop blueboat_pid --ros-args \
  -p target_heading_rad:=0.7 -p surge_command:=0.20
```

`blueboat_pid.py` subscribes to `/blueboat/odom` and uses yaw feedback to maintain a **single desired heading**. It publishes `geometry_msgs/msg/Twist` on `/blueboat/cmd_vel`. The default `surge_command` is **0.0** (stationary). This is **not** a complete lap or gate-following controller; participants must decide which heading and speed to request from observed sensors.

### 6.3 Camera-based steering starter · six `TODO`s

File: `roscon_stonefish_workshop/blueboat_camera_steering.py`.

The supplied starting point does the ROS 2 boilerplate: subscribe to `/blueboat/camera/image_raw/image_color` using sensor-data QoS, convert with `cv_bridge`, segment candidate yellow objects, optionally detect `DICT_4X4_50` ArUco IDs **11–22** and **31–42**, calculate a simple horizontal pixel error and publish a Twist command. It stops on **stale frames, missing targets or disabled motion**.

The current two-largest-yellow-blobs heuristic is intentionally incomplete. Finish these source-code exercises:

| TODO | Participant task | Why it matters |
|:--:|:--|:--|
| **1** | Calibrate the HSV threshold and reject false yellow detections. | Reflections and marker headers can look like buoys. |
| **2** | Associate the correct **left and right** buoy of the next gate. | Two large blobs may be from different gates or the same boundary. |
| **3** | Track candidate buoys and marker IDs over multiple frames. | Stabilises control through occlusion, motion and turning. |
| **4** | Determine checkpoint progression **from live observations**. | Prevents continually chasing the wrong gate. |
| **5** | Implement adaptive forward speed and safer cornering. | Avoids overshoot and excessive penalties. |
| **6** | Verify camera mirroring, yaw sign and steering direction. | Prevents turning away from a visible target. |

Run **without motion** first:

```bash
ros2 run roscon_stonefish_workshop blueboat_camera_steering
```

After inspecting the log and camera:

```bash
ros2 run roscon_stonefish_workshop blueboat_camera_steering --ros-args \
  -p enable_motion:=true -p surge_speed:=0.15
```

**Forward direction correction:** A positive `/blueboat/cmd_vel` `linear.x` now maps to positive raw motor outputs using the corrected `blueboat_thruster_adapter.py` (`surge_sign: -1.0` in the adapter's internal conversion). For straight motion with `linear.x=0.15` and `angular.z=0.0`, the expected raw outputs are `[0.15, 0.15]`:

```bash
ros2 topic echo /blueboat/controller/thruster_setpoints_sim
```

Do **not** fix reversed motion by inserting a negative `surge_speed` in the camera controller. Verify that you installed the latest thruster adapter; its positive-forward convention should apply to all Twist controllers.

### 6.4 Optional standalone ArUco detector

```bash
ros2 run roscon_stonefish_workshop blueboat_aruco_detector
ros2 topic echo /blueboat/aruco/ids
```

ArUco detection requires an OpenCV build exposing `cv2.aruco`. Marker visibility depends on camera pose, rendered textures and image resolution. The **judge**, not this detector, determines official checkpoint crossings.

### 6.5 Reinforcement learning and other approaches

You may develop a learned or hybrid controller from live RGB/ArUco, odometry, IMU/GPS and previous actions. Use Practice Mode for training episodes. A ready-made Gymnasium training environment or pretrained policy is **not provided**. Learning from hidden race coordinates, judge oracles or precomputed routes violates fair play, even if the final policy only publishes Twist commands.

---

## 7. Submitting results and local leaderboard

After your official attempts, export **all recorded attempts** (not only the best one):

```bash
ros2 run roscon_stonefish_workshop race_export --team Team_01
```

Default export location:

```text
~/roscon_submissions/Team_01_<course-hash>_submission.zip
```

The persistent local judge ledger is under:

```text
~/.ros/roscon_blueboat_grand_prix/
  Team_01/
    <first-12-hex-of-course-hash>/
      attempt_01/
        report.json
        trajectory.csv
      attempt_02/
        report.json
        trajectory.csv
      attempt_03/
        report.json
        trajectory.csv
```

Each `report.json` records the attempt number/ID, start/checkpoint information, status and (for a completed run) elapsed time, penalties and adjusted time. Trajectories are recorded for review. `race_export` includes a `submission_summary.json` containing the best completed result; if no attempt finished, the best time is `null`.

To display a leaderboard from submitted ZIPs in a directory:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 scripts/show_blueboat_leaderboard.py ~/roscon_submissions
```

**Comparison requirements:** Use the same course, judge and physics settings; a different course hash is not directly comparable. Local results and ZIP hashes support review but are **not cryptographically tamper-proof**. Organisers may request source code or reproduce a submission on an organiser-controlled installation.

---

## 8. BlueROV2: photogrammetry and reconstruction

**Independent of the BlueBoat competition.** The dedicated underwater site includes a collidable heightmap seabed, survey lighting, and three targets: a weathered concrete **cat**, marble **bust** and **shipwreck**. The simulated BlueROV2 Heavy retains its supplied physical geometry and eight-thruster configuration.

| Level | Target | Suggested acquisition |
|:--|:--|:--|
| Basic | Cat | One or two close-range orbits, several camera heights and 60–80% overlap. |
| Intermediate | Marble bust | Multiple elevations and converging views; account for low-texture marble. |
| Advanced | Shipwreck | Overlapping parallel passes, both hull sides and elevated oblique views. |

### 8.1 Launch and check sensors

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py
ros2 topic list | grep bluerov
ros2 topic hz /bluerov2/camera/image_raw/image_color
```

The camera publishes at `/bluerov2/camera/image_raw/image_color`. Controller odometry is `/bluerov2/odometry`; a standard Stonefish IMU is at `/bluerov2/imu/data`, with simulated DVL and pressure topics also available. Camera capture is not a pre-rendered texture dataset: collect images while the vehicle moves.

### 8.2 Manual motor demo (controller off)

**Terminal A:**

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py controller:=false
```

**Terminal B:**

```bash
ros2 run roscon_stonefish_workshop bluerov_thruster_demo --demo

# Or test one axis at low power:
ros2 run roscon_stonefish_workshop bluerov_thruster_demo \
  --axis heave --value 0.25 --duration 2
```

The eight motor order is `FrontRight, FrontLeft, BackRight, BackLeft, DiveFrontRight, DiveFrontLeft, DiveBackRight, DiveBackLeft`. Commands are normalised and simulator-specific. **Do not run the motor demo while the waypoint controller is active.**

### 8.3 Waypoint PID: TURN → MOVE → SETTLE

Start an obstacle-free scenario with the controller and **without** autonomy:

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  controller:=true autonomy:=false
```

In another terminal, run **one** movement test at a time:

```bash
# Forward movement
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --forward 2 --seconds 45

# Yaw response (separate test)
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --turn-deg 30 --seconds 30

# Descend 0.4 m in world NED (separate test)
ros2 run roscon_stonefish_workshop bluerov_waypoint_demo --down 0.4 --seconds 35
```

Controller status and odometry:

```bash
ros2 topic echo /bluerov2/waypoint_pid/status
ros2 topic echo /bluerov2/odometry
```

**Updated controller profile** in `config/bluerov_pid.yaml`:

| Parameter | Value | Purpose |
|:--|--:|:--|
| `max_forward` | **0.58** | Long-transit surge command ceiling. |
| `kp_forward` | **0.28** | Forward distance feedback. |
| `kp_yaw` | **0.38** | Faster heading correction. |
| `kd_yaw` | **0.22** | Yaw damping. |
| `max_yaw` | **0.34** | Faster turning authority. |
| `kp_depth` | **1.20** | Stronger dive response. |
| `ki_depth` / `kd_depth` | **0.10 / 0.30** | Depth correction. |
| `max_heave` | **0.50** | Vertical command limit, increased from 0.20. |
| `max_thruster_setpoint` | **0.65** | Maximum absolute individual motor command. |

Depth **Z increases downward** in world NED. The controller's initial `yaw_sign` and `heave_sign` are `-1.0`; confirm them by observing Stonefish. A wrong-direction yaw guard can reverse yaw polarity once and then stop yaw/forward commands if divergence persists. These safeguards are not a substitute for actual motor testing.

To run the supplied combined movement test:

```bash
ros2 run roscon_stonefish_workshop bluerov_waypoint_test \
  --timeout 55 \
  --output ~/roscon_photogrammetry/waypoint_pid_test.json
```

For custom waypoints, publish `geometry_msgs/PoseStamped` to `/bluerov2/setpoint` in the controller's **world NED** coordinate convention. Do not publish competing goals simultaneously. A goal must remain fresh; the controller stops when its input or odometry is stale.

### 8.4 Record images

Start the scene with automatic capture:

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  controller:=true capture:=true autonomy:=false
```

Or, if you launched with `capture:=false`, start the recorder separately:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_capture
```

**Do not start two recorders.** PNGs are written to `~/roscon_photogrammetry/survey/images/`; `manifest.csv` includes timestamps and available odometry for diagnostics. Images are sampled at least 1 s apart, preferring translation-separated camera positions. Aim for **30–80 overlapping views as a first experiment**, not repeated still frames.

### 8.5 Reconstruct with COLMAP

After capturing images:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
bash scripts/run_photogrammetry_colmap.sh \
  ~/roscon_photogrammetry/survey/images \
  ~/roscon_photogrammetry/colmap_result
```

The supplied pipeline uses approximate pinhole intrinsics for the 960×720 simulated camera; calibrate or refine them for serious evaluation. Sparse reconstruction may fail with poor overlap, low texture or insufficient viewpoint diversity. COLMAP's raw monocular reconstruction is **not inherently metric or aligned to Stonefish world coordinates**.

### 8.6 Evaluate image acquisition

Cat + bust:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge statues
```

Shipwreck:

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge wreck
```

Or both: `--challenge both`. These **0–100 acquisition scores** reward image count (25), spatial/azimuthal viewpoint coverage (60) and multiple height/depth bands (15). They are **survey-planning proxies**, not a measurement of COLMAP accuracy or proof that the target is visible in every photo. Geometry evaluation is optional and separate.

### 8.7 Optional aligned-geometry evaluation

With a reconstructed PLY point cloud that has been **metric-scaled, correctly aligned to the Stonefish world NED frame and cropped to the target**, run:

```bash
python3 -m pip install scipy trimesh

ros2 run roscon_stonefish_workshop photogrammetry_geometry_eval \
  --target cat --cloud /path/to/aligned_cat.ply \
  --reference-dir \
  ~/ros2_ws/src/roscon_stonefish_workshop/data/photogrammetry
```

The evaluator reports nearest-neighbour precision, completeness and F-score at a specified distance threshold (default 0.10 m for statues; 0.25 m for the wreck). If COLMAP registered at least six camera frames, `photogrammetry_align` can estimate a reconstruction-to-world transform from the saved `manifest.csv` and COLMAP's converted `images.txt` for **evaluation only**. Inspect an aligned cloud overlay before interpreting scores.

### 8.8 World and lighting notes

The synthetic terrain is a collidable **88 × 88 m** heightmap, with the original sand JPG texture (not a premade orthomosaic). Static fill lights illuminate the survey scene and may be visible as white spheres. The two vehicle-mounted survey lamps were removed in the current workshop world. The supplied BlueROV2 retains its original Heavy physical components and standard simulated IMU; confirm neutral buoyancy by observing depth drift before tuning ballast. Scene assets and textures are stored under `data/photogrammetry/`.

---

## 9. BlueROV2: autonomous inspection

This third challenge uses an alternative world with **11 solid obstacles** and the same three targets. It introduces route feasibility, camera occlusions and next-best-view choices. It has *different* evaluation rules from the BlueBoat Grand Prix.

### 9.1 Cat: map-assisted demonstration

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=cat autonomy_mode:=reference
```

The cat reference uses a **known static obstacle/target map**, A* transit and approximate occlusion-aware inspection viewpoints before returning home. It is a worked mapping/planning reference—not online semantic object discovery or learned vision.

Monitor:

```bash
ros2 topic echo /bluerov2/inspection/status
ros2 topic hz /bluerov2/camera/image_raw/image_color
```

### 9.2 Bust or wreck: deliberately unfinished starter

```bash
# Start the bust SEARCH-only exercise
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=bust autonomy_mode:=starter
```

Run the wreck separately with `target:=wreck autonomy_mode:=starter`. The starter stops at `STARTER_READY` after its search prefix. There is **no** fully solved `reference` mode for bust or wreck.

Development hooks in `roscon_stonefish_workshop/autonomy_exercises.py` include `search_waypoints`, `discover_target`, `update_obstacles`, `plan_safe_route`, `choose_next_best_views` and `should_return_home`.

### 9.3 Execute your own inspection plan

An intermediate route is to author waypoints and use the map-assisted **custom** executor:

```bash
cp ~/ros2_ws/src/roscon_stonefish_workshop/data/autonomy/bust_plan_TEMPLATE.json \
  ~/bust_plan.json
# Edit ~/bust_plan.json and fill in YOUR waypoints.

ros2 run roscon_stonefish_workshop autonomy_plan_check \
  --target bust --plan ~/bust_plan.json

ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py \
  scenario:=bluerov2_autonomy_obstacles.scn \
  controller:=true capture:=true autonomy:=true \
  target:=bust autonomy_mode:=custom plan_file:=$HOME/bust_plan.json
```

Plan entries contain world-NED `x`, `y`, `z` (metres), `yaw` (radians) and `phase` (such as `SEARCH`, `APPROACH`, `INSPECT`). The validator checks geometric plan feasibility and viewpoint coverage. For the wreck, use `wreck_plan_TEMPLATE.json` and replace `bust` with `wreck`.

This **known-map custom-plan mode is allowed in the BlueROV2 inspection exercise**; it is **not** an exception to the BlueBoat Grand Prix's no-course-file rule. The advanced version requires camera-based target discovery and sensor-derived local obstacle representations instead of using the known map.

Optional visual detections may be published as `std_msgs/msg/String` JSON on `/bluerov2/autonomy/detections`, e.g. `{"target":"bust","detected":true,"confidence":0.8}`. Enabling `visual_required` without a functioning detector prevents progression; do not simulate detections by sending invented success messages.

### 9.4 Inspection evaluation

```bash
ros2 run roscon_stonefish_workshop photogrammetry_evaluate \
  --survey ~/roscon_photogrammetry/survey --challenge both

ros2 run roscon_stonefish_workshop inspection_evaluate \
  --mission ~/roscon_photogrammetry/missions/inspection_bust_YYYYMMDD_HHMMSS.json
```

The inspection proxy awards up to 100 points across approach (15), inspection (35), return (25), completion efficiency (10) and optional acquisition (15), with a **20-point proxy penalty** for violations of sampled keep-out clearance. These are **map-based proxies**, not true 3D collision-event detection or measured surface completeness.

---

## 10. Troubleshooting

| Problem | First check |
|:--|:--|
| **BlueBoat moves backward when `linear.x > 0`** | You may have the old thruster adapter. In the corrected release `linear.x=+0.15`, yaw=0 gives `[+0.15, +0.15]` on `/blueboat/controller/thruster_setpoints_sim`. Restart the simulator after rebuilding. |
| BlueBoat does not move with the direct demo | Launch with `thruster_adapter:=false`; do not run two publishers to the same motor topic. |
| BlueBoat camera controller does nothing | Check `enable_motion` (false by default), image rate, QoS, and whether two plausible yellow targets are found; stale/missing targets deliberately publish zero Twist. |
| BlueBoat steers toward the wrong buoys | The camera example is intentionally incomplete; implement gate association and tracking in `TODO`s 2–4. |
| `FAIR PLAY VIOLATION` appears | Remove restricted route/waypoint parameters or private judge-topic subscriptions from **participant navigation**. Use live sensors. The judge can still use organiser configuration. |
| `race_start` is rejected | Confirm a judge is running, odometry exists, the boat is within 1.5 m of spawn, not already armed, and the team has attempts remaining. |
| START does not begin timing | The judge must be armed **before** the forward START crossing. Practice crossings do not count. |
| FINISH does not produce a completed result | Confirm that START and all 12 checkpoints were crossed **in order**, then FINISH. |
| Attempts unexpectedly show fewer remaining | An attempt was already **successfully armed** and its report persisted. Check the team's existing `attempt_*/report.json`; do not delete them. |
| `race_export` says no records | Check the exact team identifier and the `results_dir` used at launch. Practice does not create official reports. |
| BlueROV2 rotates indefinitely | Test +30° yaw in the obstacle-free world; check `/bluerov2/waypoint_pid/status`, `yaw_sign` and extra motor publishers. |
| BlueROV2 fails to descend | Check world NED convention (positive Z is down), `heave_sign`, `max_heave: 0.50` and controller status. |
| Missing textures, black objects or simulator abort | Check `data/photogrammetry/textures`, mesh normals/UVs, installed model files and Stonefish logs. |
| An offline test passes but the robot behaves incorrectly | Unit tests do not exercise Stonefish's actual rendered camera, physics or thruster orientation. Test live. |

Useful commands:

```bash
ros2 topic list | grep -E 'blueboat|bluerov'
ros2 topic info -v /blueboat/cmd_vel
ros2 topic info -v /blueboat/controller/thruster_setpoints_sim
ros2 topic echo /blueboat/race/status
ros2 topic echo /bluerov2/waypoint_pid/status
```

---

## 11. Project layout, limitations and licences

```text
roscon_stonefish_workshop/
├── README.md
├── package.xml · setup.py · setup.cfg
├── launch/
│   ├── blueboat_workshop.launch.py
│   └── bluerov2_workshop.launch.py
├── config/
│   ├── blueboat_race.json             # organiser geometry for scene/judge
│   ├── FAIR_PLAY_NOTICE.txt          # live-sensor competition rules
│   └── bluerov_pid.yaml              # tuned waypoint controller
├── data/
│   ├── scenarios/                    # BlueBoat and BlueROV2 worlds
│   ├── robots/                       # vehicle models
│   ├── textures/                     # ArUco and visual markers
│   ├── photogrammetry/               # terrain, objects, textures
│   └── autonomy/                     # inspection plan templates
├── roscon_stonefish_workshop/
│   ├── blueboat_pid.py               # sensor-only heading PID starter
│   ├── blueboat_camera_steering.py   # vision + TODOs (incomplete)
│   ├── blueboat_thruster_adapter.py  # positive forward Twist convention
│   ├── blueboat_aruco_detector.py
│   ├── fair_play.py                  # educational guards for supplied APIs
│   ├── blueboat_race_monitor.py      # local official judge
│   ├── race_engine.py / race_geometry.py / race_records.py
│   ├── bluerov_pid.py
│   └── inspection_reference.py / autonomy_exercises.py
├── scripts/
│   ├── generate_blueboat_race.py     # organiser scene generation
│   ├── run_photogrammetry_colmap.sh
│   └── show_blueboat_leaderboard.py
└── tests/
```

**Event integrity.** The judge needs private geometry and keeps local scoring enabled, while the supplied participant interfaces issue a violation and stop if asked to load forbidden route files. Anyone who controls their computer can inspect or modify local files; therefore **neither the fair-play guard nor the three-attempt counter is tamper-proof**. Use a consistent course/physics setup and organiser review for ranked prizes.

**Licensing and sharing.** The ROS package declares `Apache-2.0` for its software. Stonefish, `stonefish_ros2`, meshes, textures and other third-party materials retain their respective licences. Please **link to the official repository instead of mirroring the full workshop asset bundle** where possible. This is a request, not an additional legal restriction overriding permissive licences.

**Simulation ≠ real robot.** Normalised thruster commands, waypoint-gain settings, nominal buoyancy and scripted safety stops are intended for Stonefish workshop exercises. Do not deploy these examples on physical vehicles without calibration, independent safety checks and suitable emergency-stop behavior.

---

**Have fun building something that can actually perceive and navigate.** The fastest legitimate BlueBoat is the one that completes the course from its sensors—not from the answer hidden in the scenario files.

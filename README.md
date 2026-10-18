# ROSCon UK 2026 — Stonefish Marine Robotics Workshop

**Two challenges · ROS 2 Humble · Stonefish 1.6 · independent laptops**

> **PUBLIC REPOSITORY — PLEASE DO NOT REDISTRIBUTE THE WORKSHOP BUNDLE**
>
> This repository is publicly accessible to support the ROSCon UK 2026 workshop.
> Please **do not copy, re-upload, mirror, or republish the complete workshop
> package** (including its custom scenarios, models, textures and event-specific
> material) elsewhere without first asking the workshop maintainers. Please
> link to the original repository instead. You are welcome to develop and share
> your **own** controllers, policies, reports and learning outcomes, subject to
> the applicable licences and rights.
>
> **Licence clarification:** This is a respectful redistribution request, not
> a technical lock or a change to the rights granted by existing licences.
> The package metadata currently identifies the Python/ROS software as
> `Apache-2.0`, which permits redistribution subject to its terms. Upstream
> Stonefish, `stonefish_ros2`, and any third-party meshes/textures retain their
> respective licences and rights. The event team must verify permission to
> publish any third-party assets before making this repository public. If
> enforceable restrictions on original workshop-only assets are desired,
> their owners must specify those rights separately; a README notice cannot
> override an existing open-source licence.

Workshop materials maintained for Michele Grimaldi and Favour Adetunji (Heriot-Watt University). This is the **single workshop README**: installation, competition rules, commands, controllers, troubleshooting and the second mapping challenge are all documented here.

> **Current status:** The BlueBoat Chiocciola Grand Prix is implemented and its **offline tests** run without ROS. The exact BlueBoat hull and 180°-pitched propellers are preserved from the working version. Graphical Stonefish and motor-sign validation must be checked on your machine. The BlueROV2 mapping challenge includes its starter algorithms and reconstruction workflow, but **the final mapping world/dataset is still pending** and will be updated separately.

## 1. The two workshop challenges

| Challenge | Robot | Student goal | Status |
| --- | --- | --- | --- |
| **1 — BlueBoat Grand Prix** | BlueBoat (two thrusters) | Complete a 226 m buoy-marked race with PID, RL or a hybrid; fastest **penalty-adjusted** finish wins. | Implemented; test locally |
| **2 — Underwater Survey and Mapping** | BlueROV2 | Plan an efficient survey, collect overlapping images, recover a 3D reconstruction with COLMAP/GLOMAP. | Starter code available; final world/data to be released later |

Each team runs **Stonefish + ROS 2 + controller + local referee on its own laptop**. No shared ROS IP, ROS_DOMAIN_ID or network is needed. Only the submitted results are collected centrally. Use the same fixed competition package and track version for every team.

## 2. Requirements and installation

Target: Ubuntu 22.04, ROS 2 **Humble**, working **Stonefish 1.6** and `stonefish_ros2`, Python 3, OpenGL 4.3-capable GPU. The workshop does not depend on COLA2. Each student/team runs its own
simulator on its own computer: a common ROS IP or organiser-hosted simulator
is not required. For optional ArUco detection, install OpenCV with its ArUco
module; for mapping, install COLMAP or GLOMAP separately. ROS dependencies include `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs` and `ament_index_python`.

**Prerequisite:** Install ROS 2 Humble and build/verify Stonefish 1.6 with
`stonefish_ros2` using their official documentation before launching this
workshop. Check that Stonefish's GUI works on your machine; a successful ROS
package build alone does not establish simulator compatibility. Helpful
resources: [ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation.html),
[Stonefish documentation](https://stonefish.readthedocs.io/en/latest/), and
[Stonefish ROS 2](https://github.com/patrykcieslak/stonefish_ros2).

For a standard Ubuntu 22.04 ROS 2 workspace, install the usual tooling if
not already present:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep unzip
```

Start from your already working Stonefish workspace or use your existing
`~/ros2_ws`. These commands replace an earlier workshop copy safely by
backing it up **outside** `src`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Unzip the supplied student ZIP (adjust the filename/path as needed)
mkdir -p /tmp/roscon_release
unzip -oq ~/Downloads/rosconUK_Stonefish_Workshop_PUBLIC_COMPLETE.zip -d /tmp/roscon_release

# If you already installed an older copy, move it outside src to avoid duplicates
[ ! -d src/roscon_stonefish_workshop ] || mv src/roscon_stonefish_workshop \
  "$HOME/roscon_stonefish_workshop_previous_$(date +%Y%m%d_%H%M%S)"

cp -a /tmp/roscon_release/rosconUK_workshop_stonefish src/roscon_stonefish_workshop
colcon build --symlink-install --packages-select roscon_stonefish_workshop
source install/setup.bash
ros2 pkg prefix roscon_stonefish_workshop
ros2 pkg prefix stonefish_ros2
```

Source `/opt/ros/humble/setup.bash` and `~/ros2_ws/install/setup.bash` in **every new terminal**. If the build cannot find `stonefish_ros2`, first build/source its workspace. Any `libStonefish` or ROS compile errors are simulator installation issues, not race-scoring issues. The previous propeller-OBJ crash was caused by missing material-library references; this package includes the corrected mesh references.

Run the offline checks from source (these do not require a running simulator):

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 -m unittest discover -s tests -v
```

## 3. Grand Prix track and rules

The **Chiocciola v4** track is about **225.8 m long**, **6.8 m wide**, with **254 fixed yellow boundary balls** (127 on each side), **12 checkpoints with ArUco signs on both sides** (24 signs), plus START and FINISH. The track is **point-to-point, not a closed lap**:

1. **20 m straight after START**, then the original S-bends.
2. A tightening **~258° clockwise snail/spiral** (*chiocciola*).
3. **10 m straight**, then the **90° elbow** (*curva a gomito*).
4. **25 m finish straight**.

Yellow buoys and ArUco signs are **non-colliding** visual track markers. ArUco IDs **11–22** are on one side and **31–42** on the other. The yellow headers indicate direction. Students can use OpenCV/vision for navigation but **ArUco detection is optional**, not the judging mechanism.

**The local referee** checks the boat's simulated ground-truth XY position, ordered virtual gates and track boundaries. Students may use `/blueboat/odom`, simulated camera, IMU and GPS for their controllers. The referee evaluates all control methods identically.

### Official run procedure and scoring

- **PRACTICE by default**: Drive anywhere, test thrusters, cross the green START line or checkpoints. **None of it counts** until you explicitly arm a run.
- **ARMED**: The local `race_start` command reserves **one of three** official attempts; boat must be near its original spawning position. The clock does **not** run yet.
- **RUNNING**: The timer starts only when the boat crosses the **green START gate forward**. All 12 checkpoints must be crossed **in order** inside their valid gate widths. Cutting the course and jumping to FINISH does not count.
- **FINISHED**: After CP12, crossing the **red FINISH gate** stops the clock and saves an official report. **DNF**: the 1,200-simulation-second time limit expires, or the active attempt is cancelled.
- Every official arm consumes an attempt **even if the team cancels or restarts**. A laptop crash may leave an incomplete `ARMED/RUNNING` report, which counts as an abandoned attempt. This is deliberate; teams should practise first.

**Adjusted time (seconds):**

\[
T_{\mathrm{adjusted}}=T_{\mathrm{elapsed}}+8N_{\mathrm{offtrack}}+2T_{\mathrm{outside}}.
\]

`N_offtrack`: number of excursions outside the track, `T_outside`: cumulative simulation-time seconds spent outside. Re-entering and leaving again incurs another +8 seconds. **Only FINISHED attempts are ranked.** Lowest adjusted time among each team's three official attempts wins. Use the same track hash/version, simulator speed/configuration and judging software for all teams.

### Why this is a time trial, not one networked multiplayer race

Every team competes asynchronously on its own machine. There is no central ROS server, and no requirement for simultaneous starts. Submit a ZIP of all local official attempts at the end; the organiser combines them into a leaderboard.

## 4. Challenge 1 — Practice Mode: free training and thruster familiarisation

**Practice Mode is free training.** Students can drive the BlueBoat, experiment
with different control algorithms and explore the circuit without affecting
their official competition results. The referee runs locally on the same laptop
and is **not armed by default**.

During practice, students may:

- Manually command the two thrusters to understand differential propulsion.
- Develop and tune a PID or waypoint-following controller.
- Test fast and slow speeds around the S-bends, tightening *chiocciola* and 90° elbow.
- Cross the **green START** line and all **12 checkpoints** as many times as they want.
- Experiment with camera vision, ArUco detection, reinforcement learning (RL)
  or combinations of PID, mapping and visual cues.
- Stop and restart the simulation to reset the BlueBoat to its spawn point.

**Nothing done in Practice Mode enters the official leaderboard.** Even a
complete, flawless START → CP01 → … → CP12 → FINISH traversal is **not an
official race** unless the team explicitly ran `race_start` **before** crossing
START. Green gates and checkpoint markers are visual navigation cues, not
proof that an attempt has been registered. Practice is unlimited and does not
consume any of the three official attempts.

### Terminal 1 — Launch Practice Mode *without* the thruster adapter

Open the first terminal on the team's own computer:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch roscon_stonefish_workshop \
    blueboat_workshop.launch.py \
    team:=Team_01 \
    thruster_adapter:=false
```

This launches the BlueBoat in Stonefish with the 254 yellow non-colliding
boundary buoys, 24 ArUco/checkpoint signs, green START and red FINISH gates,
and the **local** race judge in PRACTICE. The adapter is intentionally disabled
so it cannot overwrite students' direct motor commands. Stop a prior simulator
instance before launching this one.

### Terminal 2 — Experiment with the two thrusters

Open a second terminal and source the ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run roscon_stonefish_workshop \
    blueboat_thruster_demo --demo
```

This simple script gives a **low-amplitude straight / stop / differential
thrust / stop** demonstration. It is a learning tool, **not an autonomous lap
controller**. The source is
`roscon_stonefish_workshop/blueboat_thruster_intro.py`.

To send a manual low-amplitude command instead:

```bash
ros2 run roscon_stonefish_workshop \
    blueboat_thruster_demo \
    --right -0.15 \
    --left -0.15 \
    --duration 3
```

This publishes normalized thruster setpoints of `-0.15` for both motors for
three seconds and sends stop commands afterwards. For safety, the demo
limits each setpoint to **[-0.3, +0.3]** and individual runs to **15 s**.
It sends `std_msgs/msg/Float64MultiArray` on:

```text
/blueboat/controller/thruster_setpoints_sim
```

**Element order:**

```text
[right_thruster, left_thruster]
```

The physical propellers in this release were pitched by 180 degrees.
**Verify by observation** which command sign moves the boat forward before
writing the controller; do not assume negative always means forward. Avoid
simultaneously publishing direct thruster commands and Twist commands through
the adapter. If the script appears to have no effect, make sure the launch
argument really is `thruster_adapter:=false`.

### Terminal 3 — Monitor the race state

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /blueboat/race/status
```

The messages contain JSON with the current judge state, checkpoint progress
and official attempts remaining. The judge should say `practice` while the
team is experimenting. Crossing green START in this state will **not** start
the official timer or save a score.

To discover the available interfaces:

```bash
ros2 topic list | grep blueboat
ros2 service list | grep /blueboat/race
```

### Switching from direct thrusters to PID / Twist control

1. Stop the direct thruster demonstration; stop Stonefish with `Ctrl+C`.
2. Restart the simulator **without** `thruster_adapter:=false` (the adapter is
   enabled by default):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop \
    blueboat_workshop.launch.py \
    team:=Team_01
```

3. In a **new terminal**, run the student PID starter:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop blueboat_pid
```

The starter intentionally commands zero forward speed until students
complete and tune its `TODO(participant)` sections. Students may instead test
the `/blueboat/cmd_vel` interface directly while the adapter is enabled:

```bash
ros2 topic pub -r 10 /blueboat/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
# Press Ctrl+C to stop publishing (the adapter has a command timeout).
```

During this phase, the students can practise as much as needed. The judge is
still in PRACTICE and the green line **does not** start a scored run.
**Only once satisfied with the controller**, restart at the initial spawn and
arm an official attempt:

```bash
ros2 run roscon_stonefish_workshop race_start
```

This command reserves **one of three official attempts**. The timer then
starts **only on a forward crossing of START**; passing all 12 checkpoints in
order and then FINISH is required for a valid score. See §7 for resetting,
DNFs and exporting results.

## 5. Writing the student PID (or an alternative controller)

Once students have completed the thruster familiarisation above, they should
work on PID (or their own RL/hybrid controller). The normal simulator launch
runs **with the adapter enabled**; direct motor demo and PID must **never run
together**.

The starter file is `roscon_stonefish_workshop/blueboat_pid.py`. Find `TODO(participant)`. It reads `/blueboat/odom` and publishes `geometry_msgs/msg/Twist` on `/blueboat/cmd_vel`; the bridge converts it to the two physical thruster setpoints, using a safety timeout. The starter intentionally leaves `surge=0`, so it will not complete the race until you add and tune propulsion, cornering and speed control.

```bash
# Launch the default race scene (adapter enabled) in Terminal A
ros2 launch roscon_stonefish_workshop blueboat_workshop.launch.py team:=Team_01
# Terminal B:
ros2 run roscon_stonefish_workshop blueboat_pid
```

Students may instead write a Python ROS 2 node subscribing to `/blueboat/odom` and/or `/blueboat/camera/image_raw` and publishing `/blueboat/cmd_vel`. **Do not** have two controller nodes publishing conflicting commands during an official run. The referee subscribes to `/blueboat/race/ground_truth_odom`; this is for scoring, not an extra required sensor for the student policy.

An optional detector is provided:

```bash
ros2 run roscon_stonefish_workshop blueboat_aruco_detector
ros2 topic echo /blueboat/aruco/ids
```

OpenCV detection reliability depends on image resolution and rendered viewpoint. It is **not required** to earn a race score.

### RL track-following pathway (optional)

A policy need not start with a global map. Before the map reveal, it can act using **local observations** (camera frames and visible buoys/ArUcos, boat heading/yaw-rate, speed, previous action). A policy can output either normalized `[surge, yaw]` commands on `/blueboat/cmd_vel` (recommended), or right/left thrusters (with the adapter disabled). Example reward terms for training: positive forward progress; penalties for time, off-track excursions and large or rapidly changing actuator commands; a strong terminal reward for an ordered finish. Keep the **official score formula unchanged**—a team's own training reward is separate from official judging.

Training needs repeated short simulated episodes: restart/reinitialise the local simulator between trials, and never rely on an unverified reset API. Use practice mode for training. Starting an official attempt only for final evaluation preserves the three available attempts.

The implementation does **not** ship a ready-to-train Gymnasium environment or a pretrained policy. RL is permitted but is more time-consuming than the baseline PID. If your RL agent uses exact centreline coordinates stored in the installed configuration, that is a map-based policy, not a map-free one.

## 6. Map release at +30 minutes (organiser publishes separately)

**There is NO automatic unlock server or countdown service.** The organiser receives a **separate ZIP** with `blueboat_race_map.png` and `blueboat_waypoints.csv`, keeps it private, and publishes both to the event GitHub repository **30 minutes after the challenge starts**, simultaneously for every team. Students refresh the repository and may use the released waypoints for map-based PID or RL.

**Fair-play limitation:** The local simulator and race judge require machine-readable track geometry, available under `config/blueboat_race.json` and encoded in the scene. The included starter PID reads this geometry. Therefore the PNG reveal is **an educational rule and shared milestone, not cryptographic protection**; teams can technically inspect the track from time zero. Ask students to respect the release timing if running a map-blind phase. For an enforceable undisclosed map, the simulation/judge would need to run in a controlled remote environment, which is outside this local-workshop design.

## 7. Three official attempts: local start, status and export

**Before every official run:** Stop/restart Stonefish so the boat is near its
original spawn position; keep the thruster adapter **enabled** and have only
one PID/RL command publisher. Source the ROS overlay in every terminal.

**Terminal 1 — launch simulator and PRACTICE referee:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch roscon_stonefish_workshop \
    blueboat_workshop.launch.py team:=Team_01
```

**Terminal 2 — run the student controller:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop blueboat_pid
```

**Terminal 3 — arm an OFFICIAL attempt locally:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run roscon_stonefish_workshop race_start
```

You can call the equivalent ROS service instead:

```bash
ros2 service call /blueboat/race/arm std_srvs/srv/SetBool "{data: true}"
```

Check the judge and attempts remaining:

```bash
ros2 topic echo /blueboat/race/status
```

**A green START crossing counts only after the judge reports ARMED.** The
clock starts on a forward START crossing, then the boat must pass CP01 through
CP12 *in order* and cross FINISH. A skipped or out-of-order checkpoint cannot
produce a FINISHED report. An off-track excursion adds 8 seconds, plus 2
seconds for every simulation second spent outside the corridor.

At the end, stop the controller, stop Stonefish, restart both for the next official attempt, then call `race_start` again. **Do not delete the local results directory between attempts.** Cancelling an active attempt also consumes the slot:

```bash
ros2 run roscon_stonefish_workshop race_start --disarm
```

Your machine stores reports under:

```text
~/.ros/roscon_blueboat_grand_prix/Team_01/<course-hash>/attempt_01/report.json
~/.ros/roscon_blueboat_grand_prix/Team_01/<course-hash>/attempt_01/trajectory.csv
```

An official report includes: team name, attempt number and ID, course SHA-256, status (FINISHED or DNF), elapsed time, penalty, adjusted time for finishers, checkpoint/event times and the recorded XY trajectory. Unfinished attempts are **not** ranked. This is a **local self-reported certificate**; it helps review results but cannot guarantee integrity when the student controls the simulator and source code.

**Export your complete competition submission:**

```bash
ros2 run roscon_stonefish_workshop race_export --team Team_01
# Writes ~/roscon_submissions/Team_01_<course-hash>_submission.zip
```

Share that ZIP with the workshop organiser by GitHub, upload form or USB. `race_export` works **offline**; Stonefish does not need to be running. It bundles all attempted results and their trajectories, with the fastest valid completed score highlighted.

**Organiser — collect result ZIPs in one directory** (e.g. `~/roscon_submissions`) and run:

```bash
cd ~/ros2_ws/src/roscon_stonefish_workshop
python3 scripts/show_blueboat_leaderboard.py ~/roscon_submissions
```

Review course hash consistency and trajectories before awarding prizes. Do not trust locally generated hashes as tamper-proof evidence. For the event, allow each team **a maximum of three official armed attempts**; practise as much as needed before arming.

## 8. Challenge 2: underwater survey and mapping (next update)

The second task is **separate from the Grand Prix**. Students work with a BlueROV2, use or adapt a low-level controller, plan a survey trajectory, collect images/poses, and reconstruct the surveyed scene with **COLMAP or GLOMAP**. The final underwater world and known-good dataset will be supplied in the next workshop revision; **do not assume the current placeholder launch loads a complete BlueROV2 mapping environment**.

**Available starter files:**

- `roscon_stonefish_workshop/bluerov_pid.py`: BlueROV2 controller starter.
- `roscon_stonefish_workshop/survey_planner.py`: survey-planning task, with `TODO(participant)`.
- `roscon_stonefish_workshop/control_utils.py` and `thruster_adapter.py`: helper/control components.
- `reconstruction/scripts/extract_images_from_bag.py`: extract recorded frames.
- `reconstruction/scripts/run_colmap.sh` and `reconstruction/scripts/run_glomap.sh`: reconstruction entrypoints; inspect arguments and adjust datasets when provided.

The exercise objective is to plan a **lawnmower or other coverage path** with adequate image overlap, baseline diversity, stand-off distance, camera direction and safe clearances, then compare coverage and reconstruction quality against survey time. Once the final world is released:

```bash
ros2 launch roscon_stonefish_workshop bluerov2_workshop.launch.py
# Placeholder currently prints a message rather than starting the finished mapping world.

ros2 run roscon_stonefish_workshop survey_planner
# In the completed environment, record actual discovered topics with:
ros2 topic list
ros2 bag record -o survey_bag <camera_topic> <odometry_topic>
```

Do not invent camera topics or claim an achievable 3D reconstruction before the BlueROV2 scene, image streams, camera calibration and ground-truth evaluation methodology are finalised. The mapping task, reconstruction commands and performance rules will be refined in the following revision.

## 9. Troubleshooting and fair-play notes

- **Stonefish aborts with `Material file not found`:** verify the installed `opti_blueboat` OBJs are the corrected package files; earlier propeller OBJ releases referenced missing `.mtl` files. Reinstall this package; inspect simulator output or use `gdb --args` if the error recurs.
- **Wrong propeller direction after 180° pitch:** inspect the two thruster commands, the SCN motor inversion and physical direction with very small test setpoints. Do not modify physics in the official shared race release without agreement.
- **Direct-thruster demo seems unresponsive:** launch with `thruster_adapter:=false`; otherwise the bridge overwrites the direct setpoints. Conversely, to use `/blueboat/cmd_vel`, **enable** the adapter.
- **`race_start` refuses to arm:** check `/blueboat/race/status`, make sure the boat is near spawn and odometry is publishing; the three official attempts may already be consumed. Restarting Stonefish resets the **boat position**, not the official-attempt ledger.
- **Green gate crossing didn't count:** in practice this is correct. Arm first, then cross START forward. Crossing virtual gates out of order or outside their width is rejected.
- **No ZIP from `race_export`:** there have been no official armed attempts for that team, or the `--team` identifier differs from the launch argument.
- **ArUcos look cropped in an older version:** use the centered-texture original-size box panels shipped in this release; do not overwrite them with experimental UV-mapped OBJ signs.
- **Simulation runs at different wall-clock speeds:** racing uses **simulation time**, not laptop wall-clock time. Fairness still depends on an identical physics scenario and rule configuration.
- **Course map before minute 30:** it is not in the student ZIP; the machine-readable geometry still exists for local simulation. Please observe the map-reveal rule in good faith.
- **Offline tests vs real simulation:** passing unit tests does not replace checking buoy rendering, camera visibility, motor signs, vehicle dynamics and correct ROS topic publication on your installed Stonefish.

## 10. Publishing the repository, attribution and sharing

This ZIP is suitable for publishing as a **public educational workshop
repository** once the event organisers have reviewed the provenance and
redistribution permissions for bundled third-party models and textures.
Please use the exact **public sharing notice at the top of this README** and
link to this repository rather than mirroring the bundle. As the package is
openly downloadable, recipients can keep a copy; this notice is a request,
not copy protection. Existing Apache-2.0 or other upstream licences govern
what recipients are legally permitted to redistribute.

Publish **only this student package** at the start of the challenge. Keep the
separate organiser map ZIP private and upload its PNG and waypoint CSV after
**30 minutes**. The course geometry is still readable in the simulator and
referee JSON; fairness for map-blind navigation relies on participants
following the shared rules. Team submissions are local ZIPs and should be
reviewed as self-reported evidence, not tamper-proof certificates.

## 11. Package layout

```text
roscon_stonefish_workshop/
├── README.md                       # the ONLY instruction/README file
├── package.xml / setup.py / setup.cfg
├── launch/                         # BlueBoat Grand Prix + mapping placeholder
├── config/blueboat_race.json       # simulator/judge track geometry (locally readable)
├── data/
│   ├── scenarios/blueboat_race.scn # fixed buoys, 12 two-sided gates, spiral track
│   ├── robots/blueboat.scn         # original BlueBoat hull / corrected thrusters
│   ├── opti_blueboat/             # original visual and collision meshes
│   └── textures/                  # full ArUco and arrow textures
├── roscon_stonefish_workshop/
│   ├── blueboat_thruster_intro.py  # direct thruster demo
│   ├── blueboat_pid.py             # student controller starter
│   ├── blueboat_race_monitor.py    # local referee
│   ├── race_engine.py / race_geometry.py / race_records.py
│   ├── race_start.py / race_export.py
│   └── bluerov_pid.py / survey_planner.py
├── scripts/show_blueboat_leaderboard.py
├── reconstruction/                # mapping workflow scripts (task 2)
└── tests/                         # ROS-free offline validations
```

**Licence and assets:** This ROS package currently declares `Apache-2.0` in
its metadata. Apache-2.0 grants redistribution rights and is **not** a
no-redistribution licence. The request above is voluntary; it does not remove
any applicable permissions. Stonefish, `stonefish_ros2`, and third-party
simulator meshes/images retain their owners' rights. Verify third-party asset
permissions before making a public GitHub release.

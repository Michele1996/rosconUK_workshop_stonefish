# Troubleshooting

## `stonefish_ros2` cannot be found

```bash
source /opt/ros/humble/setup.bash
cd ~/roscon_ws
source install/setup.bash
ros2 pkg prefix stonefish_ros2
```

If the package is missing, confirm that Patryk Cieślak's upstream
`stonefish_ros2` repository is inside `~/roscon_ws/src` and rebuild.

## Stonefish does not open / OpenGL error

Stonefish requires OpenGL 4.3+ and appropriate GPU drivers.

Check:

```bash
glxinfo | grep "OpenGL version"
```

On laptops with hybrid graphics, make sure the simulator is using the intended
GPU.

## Workshop package cannot be found

```bash
cd ~/roscon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix roscon_stonefish_workshop
```

## Scenario or workshop asset cannot be found

The BlueBoat preparation exercise is self-contained in this repository under
`data/`. It should not require another simulation-data package.

Rebuild and source the workspace:

```bash
cd ~/roscon_ws
colcon build --symlink-install
source install/setup.bash
```

## Simulation is slow

Reduce the Stonefish rendering resolution/quality in the launch file, close
other GPU-heavy applications, and verify that the dedicated GPU is active.

## BlueROV2 final environment is missing

This is expected in the preparation release. The final survey environment and
visual assets will be added before the workshop.

## COLMAP/GLOMAP is unavailable

Reconstruction software is optional pre-work. A known-good image sequence will
be provided for the workshop so that reconstruction can still be completed.

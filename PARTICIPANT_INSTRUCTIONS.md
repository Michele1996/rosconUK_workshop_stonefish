# Participant Instructions

## Underwater Robotics with ROS and Stonefish: Simulate, Explore!

### Before ROSCon

Please complete the installation in `INSTALL.md` and run the verification in
`PRE_WORK.md`. The required preparation exercise is the BlueBoat scene.

### What you will implement

You will receive starter code with explicit `TODO(participant)` sections:

1. `blueboat_pid.py` — complete/tune a simple goal-reaching PID controller.
2. `survey_planner.py` — design the BlueROV2 survey path.

The low-level BlueROV2 PID is provided so the final challenge focuses on
planning and data acquisition rather than spending the workshop implementing an
8-thruster controller.

### Reconstruction

During the final exercise you will:
1. record the RGB stream in a ROS 2 bag;
2. extract images with `reconstruction/scripts/extract_images_from_bag.py`;
3. run COLMAP or GLOMAP;
4. inspect registered cameras and reconstruction coverage.

The final underwater environment and known-good reference image sequence will
be added before the workshop.

### Bring

Laptop, charger, and (recommended) a mouse. A dedicated GPU with OpenGL 4.3+
support is recommended for Stonefish graphical simulation.

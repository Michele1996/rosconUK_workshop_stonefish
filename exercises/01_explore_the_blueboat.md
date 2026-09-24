# Exercise 1 — BlueBoat PID Challenge

Launch the BlueBoat scene and inspect its ROS interfaces.

Your target is the **red cube**.

Open:

`roscon_stonefish_workshop/blueboat_pid.py`

Find `TODO(participant)`. Complete the forward-speed portion of the controller,
then tune the distance and heading loops.

Questions:
- What happens if heading control is too weak?
- Why reduce surge when heading error is large?
- How does first-order thruster response change the behaviour compared with an
  instantaneous actuator?

Instructor reference: `scripts/blueboat_pid_solution.py`.

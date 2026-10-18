# Exercise 3 — Survey Planning Challenge

Open:

`roscon_stonefish_workshop/survey_planner.py`

Find `TODO(participant)` and create a survey path.

Your goal is **not simply to visit every part of the scene**. Your trajectory
must generate an image sequence suitable for Structure-from-Motion.

Consider:
- image overlap;
- stand-off distance;
- line spacing;
- camera orientation;
- number of turns;
- mission time.

A lawn-mower path is a good baseline, but you may improve it.

Record the RGB stream and navigation data with `ros2 bag record`.

Instructor reference: `scripts/survey_planner_solution.py`.

The exact bounds will be updated when the final underwater environment is
provided.

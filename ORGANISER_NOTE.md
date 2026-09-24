# ROSCon organiser submission material

Participant-facing preparation material is included in this repository:
- `README.md`
- `PARTICIPANT_INSTRUCTIONS.md`
- `INSTALL.md`
- `PRE_WORK.md`
- `TROUBLESHOOTING.md`
- `WORKSHOP_SCHEDULE.md`

Dependencies are intentionally limited to:
1. ROS 2 Humble;
2. Patryk Cieślak's upstream Stonefish;
3. Patryk Cieślak's upstream `stonefish_ros2`;
4. this workshop repository.

There is no `cola2_stonefish` dependency.

Before the final public release:
- pin the tested Stonefish + stonefish_ros2 commit/tag pair;
- add Michele's final BlueROV2 survey environment and its required visual assets;
- set the final RGB topic and survey bounds;
- generate the known-good reconstruction image sequence.

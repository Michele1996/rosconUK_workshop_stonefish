BlueBoat balanced box-physics diagnostic
=======================================

Files:
- blueboat_box_physics_balanced.scn
- opti_blueboat/blueboat_hull_box_phy.obj

The original right_side_or.obj and left_side_or.obj are NOT included.
They stay in your existing Stonefish data/opti_blueboat/ directory,
and remain the VISUAL meshes. Only the PHYSICAL meshes become boxes.

Copy:
  cp blueboat_box_physics_balanced.scn <your_scenario_directory>/
  cp opti_blueboat/blueboat_hull_box_phy.obj <your_stonefish_data_directory>/opti_blueboat/

Both hulls use identical box geometry (1.139003 x 0.1717835 x 0.358401 m)
with equal 4kg masses, symmetric Y placement (+/- 0.34738925 m),
and matching SUBMERGED physics modes. The rest of the vehicle looks,
structural geometry, buoyancy cylinders, sensors and ROS topics were kept.

This is a symmetric *diagnostic approximation*, not a validated BlueBoat
hydrodynamic model. Equal rpm does not imply exact equal thrust under flow.
Be sure the thruster bridge is not also publishing on the command topic.
Run left-only and right-only from zero yaw in fresh simulations; check
that their yaw accelerations have opposite signs.

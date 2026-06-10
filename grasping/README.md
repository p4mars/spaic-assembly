# grasping
Tile pick-and-place with MoveIt and a custom analytic 4-DOF inverse kinematics.

Run:
```bash
# For real robot
ros2 launch grasping grasping.launch.py
# For simulation
ros2 launch grasping grasping.launch.py sim:=true
```

Call actions/services:
```bash
ros2 action send_goal /pick_tile grasping/action/PickTile "{pos: {header: {frame_id: base_link}, point: {x: 0.37, y: 0.0, z: -0.1}}}" --feedback

ros2 action send_goal /drop_tile grasping/action/DropTile "{pos: {header: {frame_id: base_link}, point: {x: 0.37, y: 0.0, z: -0.1}}}" --feedback

ros2 service call /move_to_detection_pose std_srvs/srv/Trigger
```

## How it works
The arm is a 4-DOF chain (`shoulder_pan`, `shoulder_lift`, `elbow`, `wrist`). The
analytic IK in [`include/grasping/custom_4dof_ik.hpp`](include/grasping/custom_4dof_ik.hpp)
holds the gripper pointing **vertically downward** so tiles are always grasped and
placed from straight above. `shoulder_pan` sets the azimuth, `shoulder_lift` + `elbow`
solve a planar 2-link problem (elbow-up branch), and `wrist` keeps the last link
vertical. The resulting joint targets are commanded with MoveIt's `setJointValueTarget`.

### Fixed/hardcoded poses
- `detection_pose`: gripper raised and pitched so the camera looks down at the
  workspace to detect tiles; the drop action returns here.
- `carry_pose`: arm tucked high above the base to carry a grasped tile safely; the
  pick action returns here.

### Action: Pick tile (`/pick_tile`)
The goal carries the tile position in `pos` (any TF frame via `pos.header.frame_id`,
e.g. `map`, `odom`, `base_link`). The server first runs IK for the tile itself **and**
for an approach point 3 cm straight above it; if either is unreachable it aborts
**before moving**. Otherwise it opens the gripper, moves to the approach point, lowers
straight down onto the tile, closes the gripper, lifts back to the approach point, and
finally moves to the `carry_pose`.

### Action: Drop tile (`/drop_tile`)
Same goal form. IK is again solved up-front for the drop position and the 3 cm
approach point above it, aborting before any motion if either is infeasible. The arm
then moves to the approach point, lowers to the drop position, opens the gripper,
retreats back up, and finally moves to the `detection_pose`.

### Service: Move to detection pose (`/move_to_detection_pose`)
A `std_srvs/srv/Trigger` service that just moves the arm to the `detection_pose`
(the same pose the drop action ends in), useful for re-detecting tiles between picks.

## Notes on joint state filter:
- The launch starts a small joint state filter that republishes from `/joint_states` to `/joint_states_filtered` and drops any `_mimic` joints.
- To avoid issues with MoveIt, the other nodes are remapped to subscribe to `/joint_states_filtered` instead of the original `/joint_states`.
- `sim` controls `use_sim_time` and the xacro sim mapping so the same nodes work in simulation and on the real robot.

Example:
```
ros2 topic echo /joint_states
header:
  stamp:
    sec: 969
    nanosec: 672000000
  frame_id: base_link
name:
- rear_right_wheel_joint
- shoulder_pan_joint
- elbow_joint
- rear_left_wheel_joint
- wrist_joint
- gripper_joint
- _Gripper_joint_r_mimic
- front_left_wheel_joint
- front_right_wheel_joint
- _gripper_link_joint_l_mimic
- shoulder_lift_joint
- _gripper_link_joint_r_mimic
position:
- 9.336975637097567e-11
- -6.26626883413195e-05
- 0.02280270127552253
- -9.336886819255596e-11
- -0.013011302811960057
- 0.01320674868660987
- -0.01318643916912432
- 1.0715694997998071e-10
- -1.0715694997998071e-10
- -0.018770416652905908
- -0.00978647119829179
- 0.01877046552191075
[...]
```
```
ros2 topic echo /joint_states_filtered 
header:
  stamp:
    sec: 954
    nanosec: 72000000
  frame_id: base_link
name:
- rear_right_wheel_joint
- shoulder_pan_joint
- elbow_joint
- rear_left_wheel_joint
- wrist_joint
- gripper_joint
- front_left_wheel_joint
- front_right_wheel_joint
- shoulder_lift_joint
position:
- 9.038902959446204e-11
- -6.266268834220767e-05
- 0.022802701275524306
- -9.038991777288174e-11
- -0.013011302811960945
- 0.013206748686606318
- 1.0373657488571553e-10
- -1.0373746306413523e-10
- -0.00978647119829179
[...]
```

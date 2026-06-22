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
ros2 action send_goal /pick_tile grasping/action/PickTile "{pos: {header: {frame_id: base_link}, point: {x: 0.31, y: 0.0, z: -0.1}}}" --feedback

ros2 action send_goal /drop_tile grasping/action/DropTile "{pos: {header: {frame_id: base_link}, point: {x: 0.31, y: 0.0, z: -0.1}}}" --feedback

ros2 service call /move_to_detection_pose std_srvs/srv/Trigger
```

Kinametics/joint constraints yield a quite narrow range of possible coordinates e.g. for base_link and straigth to the front it works from {x: 0.28, y: 0.0, z: -0.1} to {x: 0.34, y: 0.0, z: -0.1}.

## Requirements
You need to add the definition of the gripper_center link and corresponding joint to `mirte-ros-packages/mirte_description/mirte_master_description/urdf/arm.xacro` file as this code expects TF2 transforms for it which are published based on it.

```
<joint name="gripper_center_joint" type="fixed">
    <origin xyz="0.0967 0 0" rpy="3.14159 0 0" />
    <parent link="wrist" />
    <child link="gripper_center" />
</joint>
<link name="gripper_center" />
```

## How it works
The arm is a 4-DOF chain (`shoulder_pan`, `shoulder_lift`, `elbow`, `wrist`). The
analytic IK in [`include/grasping/custom_4dof_ik.hpp`](include/grasping/custom_4dof_ik.hpp)
holds the gripper pointing **vertically downward** so tiles are always grasped and
placed from straight above. `shoulder_pan` sets the azimuth, `shoulder_lift` + `elbow`
solve a planar 2-link problem (elbow-up branch), and `wrist` keeps the last link
vertical. The resulting joint targets are commanded with MoveIt's `setJointValueTarget`.

The initial approch position (3 cm above the target position) is not used anymore as it did not really provide any benefit due to the low precision of the arm.
The code is still present in the `grasping.cpp` and just commented out.

### Fixed/hardcoded poses
- `detection_pose`
- `carry_pose`

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

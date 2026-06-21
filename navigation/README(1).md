# Navigation Module — Tetris Assembly

This folder contains the navigation logic for the Mirte Master robot in the
tetris-assembly project. Its job is to translate "go to location X" requests
from the orchestrator (or any caller) into actual movement, using Nav2 for
path planning, obstacle avoidance, and motion control.

The core piece is `move_to_server.py`, which:

- Subscribes to live marker detections from the vision team and stores each
  detection as a named, world-anchored location (in the `map` frame).
- Exposes a `/move_to` service that takes a label (e.g. `'pickup'`, `'goal'`)
  and dispatches the robot to that location via Nav2.
- Falls back to a YAML file of static locations if a label has not been
  detected at runtime.
- Provides two helper services for fine-grained motion: `/move_forward`
  (drive a number of meters along the current heading) and `/rotate_ccw`
  (rotate in place by a number of degrees).
- Publishes status messages on `/robot_status` so callers can know when a
  navigation succeeded or failed.


## Interfaces

| Direction | Topic / Service / Action            | Type                                      | Notes |
| --------- | ----------------------------------- | ----------------------------------------- | ----- |
| In  (sub) | `/detection/marker_pose`            | `geometry_msgs/msg/PoseStamped`           | Latest detected pose, encodes marker id in `position.z`. Cached on every message. |
| In  (sub) | `/detection/found_marker_id`        | `std_msgs/msg/Int32`                      | Currently unused — id is read from the pose's z field instead. Kept subscribed for forward compatibility. |
| In  (sub) | `/amcl_pose`                        | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current robot pose from AMCL. Used by `/move_forward` and `/rotate_ccw`. |
| In  (srv) | `/move_to`                          | `mirte_location_markers/srv/MoveTo`       | `{location: string}` → `{success: bool, message: string}` |
| In  (srv) | `/move_forward`                     | `mirte_location_markers/srv/MoveForward`  | `{meters: float}` → `{success: bool, message: string}` |
| In  (srv) | `/rotate_ccw`                       | `mirte_location_markers/srv/RotateCCW`    | `{degrees: float}` → `{success: bool, message: string}` |
| Out (pub) | `/robot_status`                     | `std_msgs/msg/String`                     | Human-readable status; orchestrator listens to this. |
| Out (act) | `/navigate_to_pose` (action client) | `nav2_msgs/action/NavigateToPose`         | Delegates motion to Nav2. |


## What gets saved, and where

Three categories of persistent data are touched by this module.

1. **The map file.** Built once per environment via `mirte_navigation`'s SLAM
   launch, saved by `nav2_map_server`'s `map_saver_cli`. Default location:
   `~/mirte_ws/src/mirte_navigation/maps/default.yaml` and `default.pgm`.
   This is the static map AMCL localizes against and Nav2 plans on top of.
   `move_to_server.py` does NOT touch this file; it is created/updated only
   when you explicitly run the SLAM + save flow.

2. **The YAML fallback locations.** Static named poses, loaded on demand
   by `move_to_server.py` when no live detection has been seen for a label.
   Location:
   `<mirte_location_markers package share>/locations/stored_poses.yaml`.
   This file ships with the `mirte_location_markers` package; it is not
   created by this module. Edit it by hand if you want pre-defined "home"
   or "park" poses.

3. **Live detections.** Stored in memory ONLY (`dynamic_locations` dict
   inside the running node) and lost when the node restarts. This is
   deliberate: detection-driven locations are session-specific and reusing
   them across runs would be more dangerous than helpful.

Nothing else is persisted. Log lines go to stdout, the Nav2 stack manages
its own caches separately.


## Normal operation (full system)

Running everything via the team's launch chain. From the PC:

```bash
ros2 launch tetris_assembly_bringup pc.launch.py
```

In parallel, from the robot (web VS Code at http://192.168.42.1:8000):

```bash
ros2 launch tetris_assembly_bringup mirte.launch.py
```

This brings up Nav2 (`mirte_navigation` minimal_navigation), the grasping
stack, and the navigation node together with the orchestrator and detection
on the PC side.

After everything is up:

1. Open RViz on the PC and confirm `/map`, `/scan`, costmaps are visible.
2. Click **2D Pose Estimate** in RViz at the robot's true position and
   orientation. AMCL needs this — without it the map and the robot's
   estimated location are not aligned and navigation goes wrong.
3. The orchestrator publishes to `/detection/target_marker_ids` to wake
   up the detector. If running things manually you do this yourself:
   `ros2 topic pub --once /detection/target_marker_ids std_msgs/msg/Int32MultiArray "{data: [0, 9]}"`.
4. The orchestrator calls `/move_to pickup`, picks the tile, calls
   `/move_to drop`, drops it, repeats.


## Manual operation (when the launch chain misbehaves)

Use this if the team's launch fails partway through, or to test the
navigation node in isolation.

Open four terminals. **In every terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/mirte_ws/install/setup.bash
```

**Terminal 1 — Nav2 stack (on the robot):**

```bash
ros2 launch mirte_navigation minimal_navigation_launch.py
```

Wait until the log output settles. Verify activation:

```bash
ros2 lifecycle get /bt_navigator     # should print:  active [3]
ros2 lifecycle get /amcl             # should print:  active [3]
```

If anything is not `active`, restart this launch — Nav2 will not respond
to goals while in `unconfigured` or `inactive` states.

**Terminal 2 — the navigation node (on the robot):**

```bash
python3 /home/mirte/mirte_ws/src/tetris-assembly/navigation/move_to_server.py
```

Expected output ends with:

```
navigate_to_pose action server is available
Services ready: /move_to, /move_forward, /rotate_ccw
```

If it hangs at `Waiting for navigate_to_pose action server...`, Nav2 in
Terminal 1 has not finished activating. Wait for it; if it never does,
restart Terminal 1.

**Terminal 3 — RViz (on the PC):**

```bash
rviz2
```

Add displays:
- `Map` ← `/map`
- `LaserScan` ← `/scan`
- `Map` ← `/global_costmap/costmap`
- `Map` ← `/local_costmap/costmap`
- `Path` ← `/plan`
- `Path` ← `/local_plan`
- `TF`
- `RobotModel`

Set Fixed Frame to `map`. Click **2D Pose Estimate** and align the robot.

**Terminal 4 — invoke navigation (anywhere on the ROS network):**

Test the service directly:

```bash
ros2 service call /move_to mirte_location_markers/srv/MoveTo \
  "{location: 'pickup'}"
```

`success: True` means the goal was accepted; arrival is reported on
`/robot_status`.

To stub a detection (instead of waiting for the real vision pipeline):

```bash
ros2 topic pub --once /detection/marker_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map', stamp: {sec: $(date +%s), nanosec: 0}},
  pose: {position: {x: 1.0, y: 0.5, z: 0.0},
         orientation: {z: 0.0, w: 1.0}}}"
```

Note: the `position.z` field carries the marker id by convention with the
vision team (the literal Z height is set to 0 inside `move_to_server.py`
before sending to Nav2 so the robot does not try to fly).


## Configuration and tuning

### Marker id ↔ location label

Top of `move_to_server.py`:

```python
MARKER_ID_TO_LABEL = {
    0: 'pickup',
    9: 'goal',
}
```

Edit these integer keys to match the ArUco IDs the vision team publishes.
No rebuild needed; restart the node and the new mapping takes effect.

### Stale-pose tolerance

```python
POSE_STALE_AFTER = Duration(seconds=60.0)
```

The window within which a cached pose is still considered valid for
commitment. Was raised during stub testing where shell-based publishers
introduce delays; can be reduced to ~1–2 seconds in production where
detections flow live with accurate timestamps.

### Nav2 parameters

Lives in `~/mirte_ws/src/mirte_navigation/params/minimal_nav2_params.yaml`.
Key parameters and what they do:

- `inflation_layer.cost_scaling_factor` — higher = sharper falloff, paths
  skim closer to walls; lower = gentler, paths stay clear of walls.
- `inflation_layer.inflation_radius` — larger safety bubble vs. tighter
  passage handling.
- `robot_radius` — must match the actual Mirte radius (~0.18 m). Wrong
  value makes every other tuning meaningless.

Edit the file, then restart `minimal_navigation_launch.py` (Nav2 reads
parameters at startup).


## Troubleshooting

| Symptom                                          | Likely cause                                           | Quick fix |
| ------------------------------------------------ | ------------------------------------------------------ | --------- |
| `'move_to' service not available`                | `move_to_server.py` not running on the network         | Start it manually (Terminal 2 above) |
| `Service call returns success but robot doesn't move` | Goal already at robot's location, or Nav2 not actually active | Check `ros2 lifecycle get /bt_navigator`; pick a goal further away |
| `move_to_server.py` hangs on startup             | Nav2 action server not advertising                     | Confirm Nav2 lifecycle is `active`; restart Terminal 1 |
| `Cached pose is X.XXs old…` warning              | Stale-pose race protection triggered                   | Increase `POSE_STALE_AFTER` for testing, or check that the vision node is timestamping correctly |
| `Failed to transform … to map`                   | TF tree broken or detection has an unknown `frame_id`  | `ros2 run tf2_ros tf2_echo map <their_frame>` to confirm |
| Robot in costmap zone, won't move                | Inflation overlaps with robot, or stale obstacle data  | `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"` |
| Robot drives but in wrong direction              | AMCL pose estimate wrong                               | Redo **2D Pose Estimate** in RViz; lidar should snap to map walls |
| `Goal aborted` / `Failed to compute control`     | Goal unreachable in costmap, or planner stuck          | Verify goal pose is in free space; try a closer/simpler goal; clear costmaps |


## Architecture notes (short version)

```
[ vision detector ] ──/detection/marker_pose──▶ [ move_to_server ]
                                                       │
[ orchestrator ]    ──/move_to (service)─────────────▶ │
                                                       │
                                                       ▼
                                            /navigate_to_pose (action)
                                                       │
                                                       ▼
                                               [ Nav2 stack ]
                                                       │
                                                       ▼
                                                   /cmd_vel
                                                       │
                                                       ▼
                                                  robot motors
```

The design intent is that the construction code should not know any
coordinates, the vision code should not know any navigation logic, and
neither should know anything about Nav2's internals. `move_to_server.py`
is the integration point that makes that possible.

Dynamic obstacle avoidance is NOT implemented here — Nav2's local costmap
and DWB controller handle that automatically because we delegate motion
to the `/navigate_to_pose` action rather than publishing raw `/cmd_vel`.

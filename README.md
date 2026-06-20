# tetris-assembly
**Group 4 of the 2026 edition of the "AE4ASM527 - Spatial AI for Industrial Automation" course at TU Delft**

<img src="mirte_grasping_tetris_tile.jpeg" width="400" alt="mirte_grasping_tetris_tile"/>

# Packages:
## data_collection (Frank) 
Provides:
- arm_teleop
- data_collector

See details in [data_collection/README.md](data_collection/README.md)

## slam (Frank)
Run with:
```bash
ros2 launch slam slam.launch.py
```

Publishes an occupancy grid on `/map_lidar` and the `map -> odom` transform using `slam_toolbox`.

Save map on the Mirte with:
```bash
ros2 run nav2_map_server map_saver_cli -t /map_lidar \
  -f /home/mirte/mirte_ws/src/mirte_navigation/maps/default
```

## navigation
TODO

## detection (Moritz)
Run with:
```bash
ros2 launch detection detection.launch.py
```
Detects ArUco markers in the gripper camera feed and publishes their pose in the map frame using `solvePnP` + TF2.

- publishes marker pose (x, y in map frame) on `/detection/marker_pose`
- publishes found marker ID on `/detection/found_marker_id`
- listens for which markers to track on `/detection/target_marker_ids`

Set target markers at runtime (can be overwritten) with:
```bash
ros2 topic pub --once /detection/target_marker_ids std_msgs/Int32MultiArray "data: [1, 2, 3, ...]"
```

See details in [detection/README.md](detection/README.md)

Also contains a Proof of Concept for Grid Detection.

<img src="grid_detection.png" width="400" alt="grid_detection"/>

## grasping (Frank)
Run with:
```bash
ros2 launch grasping grasping.launch.py
```

Providing `/pick_tile`, `drop_tile` actions and the `move_to_detection_pose` service

See details in [grasping/README.md](grasping/README.md)

## orchestration (Frank)
Run with:
```bash
ros2 launch orchestration orchestrator.launch.py
```

- contains the mapping of Aruko/tile ID to target position in `TILE_TARGETS` at the top
- calls other services/actions (nodes need to be running)
- robust to failure of steps through retrying, skipping (or quitting) the individual navigation, detection and grasping steps
- to retry: `ros2 topic pub --once /orchestrator/control std_msgs/String "data: 'r'"`
- to skip: `ros2 topic pub --once /orchestrator/control std_msgs/String "data: 's'"`
- to quit: `ros2 topic pub --once /orchestrator/control std_msgs/String "data: 'q'"`

## bringup (Frank)
Launch files for easier startup and defining recommended machine where the node should run (although that is flexible):
```bash
ros2 launch bringup mirte.launch.py # on the Mirte Robot via SSH
ros2 launch bringup pc.launch.py # on the PC/laptop (ROS2 Humble, same network and ROS_DOMAIN_ID as Mirte required)
```
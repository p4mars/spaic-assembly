# tetris-assembly
**Group 4 of the 2026 edition of the "AE4ASM527 - Spatial AI for Industrial Automation" course at TU Delft**

TODO: Add picture

# Packages:
## slam
Run with:
```bash
ros2 launch slam slam.launch.py
```

Publishes an occupancy grid on `/map_lidar` and the `map -> odom` transform using `slam_toolbox`.

save map on the Mirte with 
```bash
ros2 run nav2_map_server map_saver_cli -t /map_lidar \
  -f /home/mirte/mirte_ws/src/mirte_navigation/maps/default
```

## detection
TODO
## grasping
TODO

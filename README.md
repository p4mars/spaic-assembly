# tetris-assembly
**Group 4 of the 2026 edition of the "AE4ASM527 - Spatial AI for Industrial Automation" course at TU Delft**

TODO: Add picture

# Packages:
## slam
- uses slam_toolbox
- publishes occupancy map under `map_lidar` and map->base_link TF2 transform

save map with 
```bash
ros2 run nav2_map_server map_saver_cli -t /map_lidar \
  -f /home/mirte/mirte_ws/src/mirte_navigation/maps/default
```

## detection
TODO
## grasping
TODO

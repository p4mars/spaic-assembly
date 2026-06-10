# Detection Node — ArUco Marker Detection & Pose Estimation

A ROS 2 node that detects ArUco markers from a camera feed, estimates their pose using `solvePnP`, and publishes their position in the map frame.

---

## Overview

```
Camera image → ArUco detection → solvePnP → TF broadcast → map lookup → PoseStamped publish
```

The node only processes markers that are:
1. Listed in the internal `MARKER_CONFIG` dictionary
2. Currently being tracked (received via `/detection/target_marker_ids`)

This means by default it detects **nothing** until you tell it which markers to look for.

---

## Node Name

```
nth_image_detection
```

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `image_topic` | string | `/gripper_camera/image_raw/compressed` | Camera image topic to subscribe to |
| `use_compressed_img` | bool | `true` | Use `CompressedImage` instead of raw `Image` |
| `camera_info_topic` | string | `/gripper_camera/camera_info` | Camera info topic (only used if `use_hardcoded_camera_info` is false) |
| `use_hardcoded_camera_info` | bool | `true` | Use hardcoded camera matrix instead of reading from topic |
| `fallback_camera_k` | double[] | `[540, 0, 320, 0, 540, 240, 0, 0, 1]` | 3×3 camera matrix (row-major). fx, fy, cx, cy |
| `fallback_camera_d` | double[] | `[0, 0, 0, 0, 0]` | Distortion coefficients [k1, k2, p1, p2, k3] |
| `process_every_n` | int | `3` | Only process every N-th frame (reduces CPU load) |
| `aruco_dict` | string | `DICT_5X5_1000` | OpenCV ArUco dictionary to use |

---

## Subscriptions

| Topic | Message Type | Description |
|---|---|---|
| `/detection/target_marker_ids` | `std_msgs/Int32MultiArray` | List of marker IDs to actively look for. Empty = nothing tracked |
| `/gripper_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera feed (compressed). Used when `use_compressed_img = true` |
| `/gripper_camera/image_raw` | `sensor_msgs/Image` | Camera feed (raw). Used when `use_compressed_img = false` |
| `/gripper_camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics. Only subscribed when `use_hardcoded_camera_info = false` |

> **Note:** The image subscription uses a QoS profile with `BEST_EFFORT` reliability and a queue depth of 1, so it always works on the freshest available frame.

---

## Publishers

| Topic | Message Type | Description |
|---|---|---|
| `/detection/marker_pose` | `geometry_msgs/PoseStamped` | Detected marker pose in the `map` frame. `header.frame_id` is set to `map_{marker_id}` |

---

## TF Frames

**Broadcasts:**
```
camera_frame → aruco_marker_{id}
```
The transform from the camera to the detected marker, computed by `solvePnP`. Published at the image timestamp.

**Looks up:**
```
map → aruco_marker_{id}
```
After broadcasting, immediately looks up the marker's position in the map frame by chaining through the full TF tree:
```
map → odom → base_link → camera_frame → aruco_marker_{id}
```

---

## Marker Config

Markers must be defined in `MARKER_CONFIG` inside the node to be processed. Currently configured markers:

| ID | Name | Size | Target Offset (marker frame) | Target Yaw |
|---|---|---|---|---|
| `0` | `tetris_grid` | 5 cm | z = 0.5 m in front | 180° (facing marker) |
| `1` | `storage_area` | 10 cm | z = 0.6 m in front | 180° (facing marker) |

> The `target_offset` and `target_yaw_offset` fields are reserved for use by a navigation node to compute where the robot should drive to when a marker is found.

---

## How To Use

**Start the node:**
```bash
ros2 run <your_package> detection_node
```

**Tell it which markers to look for:**
```bash
# Look for marker ID 0 only
ros2 topic pub --once /detection/target_marker_ids std_msgs/Int32MultiArray "data: [0]"

# Look for markers 0 and 1
ros2 topic pub --once /detection/target_marker_ids std_msgs/Int32MultiArray "data: [0, 1]"
```

**Monitor detections:**
```bash
ros2 topic echo /detection/marker_pose
```

---

## Dependencies

- `rclpy`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `cv2` (OpenCV with ArUco support)
- `cv_bridge`
- `tf2_ros`
- `numpy`
- `scipy`

---

## Known Limitations

- The TF transform is only published every N-th frame (`process_every_n`). If the robot moves between frames, the marker position in the map may briefly be stale.
- Camera calibration is not performed automatically. In simulation, the camera info topic publishes all zeros, so `use_hardcoded_camera_info = true` is recommended.
- The node does not filter or average detections over time. A noisy or partially visible marker may produce unstable pose estimates.
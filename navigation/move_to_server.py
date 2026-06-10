#!/usr/bin/env python3
# ROS2 Humble
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from mirte_location_markers.srv import MoveTo
from mirte_location_markers.srv import MoveForward
from mirte_location_markers.srv import RotateCCW

# --- TF for transforming detections to the map frame ---
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  # registers PoseStamped support on the buffer


class MoveToLocation(Node):
    # === EDIT THIS to match what the vision teammate is sending. ===
    # Maps a detected marker ID (Int32) to the human-readable label
    # that the /move_to service consumes. Add entries for whatever
    # markers your construction task uses.
    MARKER_ID_TO_LABEL = {
        1: 'pickup',
        2: 'goal',
    }
    # Discard a cached pose if it's older than this when the marker-id
    # announcement arrives (race protection).
    POSE_STALE_AFTER = Duration(seconds=1.0)
    # ================================================================

    def __init__(self):
        super().__init__('move_to_location_node')

        # Resolve poses YAML inside this package (kept as static fallback)
        self.package_share = get_package_share_directory('mirte_location_markers')
        self.locations_dir = os.path.join(self.package_share, 'locations')

        # Status publisher
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('navigate_to_pose action server is available')

        # Cache current robot pose from AMCL (map frame)
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10
        )

        # --- TF buffer and listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Live store of detected named locations. ---
        # Keys are labels ('pickup', 'goal'); values are PoseStamped in 'map'.
        self.dynamic_locations: dict[str, PoseStamped] = {}

        # --- Detection interface (two-topic pattern from vision teammate) ---
        # /detection/marker_pose streams the latest detected pose continuously.
        # We just cache it; we don't know which marker it belongs to yet.
        self._latest_pose: PoseStamped | None = None
        self.create_subscription(
            PoseStamped, '/detection/marker_pose', self._on_marker_pose, 10
        )
        # /detection/found_marker_id fires when a specific marker has been
        # detected. THAT is the trigger to commit the cached pose to the
        # corresponding label in dynamic_locations.
        self.create_subscription(
            Int32, '/detection/found_marker_id', self._on_marker_found, 10
        )

        # Services
        self.srv_move_to = self.create_service(MoveTo, 'move_to', self.handle_move_to)
        self.srv_move_forward = self.create_service(MoveForward, 'move_forward', self.handle_move_forward)
        self.srv_rotate_ccw = self.create_service(RotateCCW, 'rotate_ccw', self.handle_rotate_ccw)
        self.get_logger().info("Services ready: /move_to, /move_forward, /rotate_ccw")

        # Track current Nav2 goal so we can cancel before sending a new one
        self.current_goal_handle = None

    # ---------- Subscribers ----------
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def _on_marker_pose(self, msg: PoseStamped):
        """Cache every pose that comes in. We commit it on the next
        /detection/found_marker_id announcement."""
        self._latest_pose = msg

    def _on_marker_found(self, msg: Int32):
        """
        A specific marker was just detected. Translate the marker id to a
        location label, take the currently cached pose, transform it into
        the map frame, and store it.
        """
        marker_id = msg.data
        label = self.MARKER_ID_TO_LABEL.get(marker_id)
        if label is None:
            self.get_logger().warn(
                f"Marker id {marker_id} has no entry in MARKER_ID_TO_LABEL; "
                f"ignoring. Add it to the mapping at the top of the file."
            )
            return

        if self._latest_pose is None:
            self.get_logger().warn(
                f"Marker {marker_id} ({label}) announced, but no pose "
                f"received on /detection/marker_pose yet — skipping."
            )
            return

        # Race protection: don't commit a pose that's much older than the
        # announcement, which would mean the streams got out of sync.
        now = self.get_clock().now()
        pose_time = rclpy.time.Time.from_msg(self._latest_pose.header.stamp)
        if (now - pose_time) > self.POSE_STALE_AFTER:
            age = (now - pose_time).nanoseconds / 1e9
            self.get_logger().warn(
                f"Cached pose is {age:.2f}s old when marker {marker_id} "
                f"({label}) was announced. Skipping."
            )
            return

        detected = self._latest_pose

        # Fast path: detection is already in map frame.
        if detected.header.frame_id == 'map':
            self.dynamic_locations[label] = detected
            self.get_logger().info(
                f"Stored '{label}' (id={marker_id}, map frame) at "
                f"({detected.pose.position.x:.3f}, "
                f"{detected.pose.position.y:.3f})"
            )
            return

        # Otherwise transform into the map frame via TF.
        try:
            map_pose = self.tf_buffer.transform(
                detected, 'map', timeout=Duration(seconds=0.5)
            )
            self.dynamic_locations[label] = map_pose
            self.get_logger().info(
                f"Stored '{label}' (id={marker_id}) at map=("
                f"{map_pose.pose.position.x:.3f}, "
                f"{map_pose.pose.position.y:.3f}) "
                f"from frame '{detected.header.frame_id}'"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to transform marker {marker_id} ({label}) from "
                f"'{detected.header.frame_id}' to map: {e}"
            )

    # ---------- YAML I/O (static fallback) ----------
    def load_locations(self, map_file_name: str):
        locations_file_path = os.path.join(self.locations_dir, f'{map_file_name}.yaml')
        if not os.path.exists(locations_file_path):
            self.get_logger().warn(f"No locations file found for {map_file_name}")
            self.get_logger().warn(f"Tried to look at {locations_file_path}")
            return {}
        with open(locations_file_path, 'r') as file:
            locations_data = yaml.safe_load(file) or {}
        if not locations_data:
            self.get_logger().warn(f"No locations data in file {locations_file_path}")
        return locations_data

    # ---------- Helpers: yaw & quaternion ----------
    @staticmethod
    def _yaw_from_quat(x, y, z, w) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _quat_from_yaw(yaw: float):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))  # x, y, z, w

    def _build_goal_from_xyyaw(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        qx, qy, qz, qw = self._quat_from_yaw(yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        return goal_msg

    def _send_goal(self, goal_msg: NavigateToPose.Goal, status_message_ok: str, status_message_rejected: str):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            finally:
                self.current_goal_handle = None

        self.status_pub.publish(String(data=status_message_ok))

        send_future = self.nav_client.send_goal_async(goal_msg)

        def _on_goal_sent(fut):
            goal_handle = fut.result()
            if not goal_handle or not goal_handle.accepted:
                self.status_pub.publish(String(data=status_message_rejected))
                return
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.move_base_done_cb)
        send_future.add_done_callback(_on_goal_sent)

    # ---------- Services ----------
    def handle_move_to(self, req, resp):
        location_label = req.location

        # Live-detection lookup first
        if location_label in self.dynamic_locations:
            stored = self.dynamic_locations[location_label]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose = stored.pose

            message_ok = f"moving to {location_label} (live detection)"
            message_rej = 'NavigateToPose goal rejected.'
            self._send_goal(goal_msg, message_ok, message_rej)
            resp.success = True
            resp.message = message_ok
            return resp

        # YAML fallback (unchanged from the original implementation)
        locations_data = self.load_locations('stored_poses')
        if location_label not in locations_data:
            message = "location unknown"
            self.status_pub.publish(String(data=message))
            resp.success = False
            resp.message = message
            return resp

        location = locations_data[location_label]
        position = location['position']
        orientation = location['orientation']
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(position['x'])
        goal_msg.pose.pose.position.y = float(position['y'])
        goal_msg.pose.pose.position.z = float(position.get('z', 0.0))
        goal_msg.pose.pose.orientation.x = float(orientation.get('x', 0.0))
        goal_msg.pose.pose.orientation.y = float(orientation.get('y', 0.0))
        goal_msg.pose.pose.orientation.z = float(orientation['z'])
        goal_msg.pose.pose.orientation.w = float(orientation['w'])

        message_ok = f"moving to {location_label}"
        message_rej = 'NavigateToPose goal rejected.'
        self._send_goal(goal_msg, message_ok, message_rej)
        resp.success = True
        resp.message = message_ok
        return resp

    def handle_move_forward(self, req, resp):
        meters = float(req.meters)
        if self.current_pose is None:
            resp.success = False
            resp.message = "No current pose available from /amcl_pose"
            return resp

        x0 = float(self.current_pose.position.x)
        y0 = float(self.current_pose.position.y)
        yaw0 = self._yaw_from_quat(
            float(self.current_pose.orientation.x),
            float(self.current_pose.orientation.y),
            float(self.current_pose.orientation.z),
            float(self.current_pose.orientation.w),
        )
        x1 = x0 + meters * math.cos(yaw0)
        y1 = y0 + meters * math.sin(yaw0)
        yaw1 = yaw0

        goal_msg = self._build_goal_from_xyyaw(x1, y1, yaw1)
        message_ok = f"moving forward {meters:.3f} m"
        message_rej = 'NavigateToPose goal rejected.'
        self._send_goal(goal_msg, message_ok, message_rej)
        resp.success = True
        resp.message = message_ok
        return resp

    def handle_rotate_ccw(self, req, resp):
        degrees = float(req.degrees)
        if self.current_pose is None:
            resp.success = False
            resp.message = "No current pose available from /amcl_pose"
            return resp

        x0 = float(self.current_pose.position.x)
        y0 = float(self.current_pose.position.y)
        yaw0 = self._yaw_from_quat(
            float(self.current_pose.orientation.x),
            float(self.current_pose.orientation.y),
            float(self.current_pose.orientation.z),
            float(self.current_pose.orientation.w),
        )
        dyaw = math.radians(degrees)
        yaw1 = self._normalize_angle(yaw0 + dyaw)

        goal_msg = self._build_goal_from_xyyaw(x0, y0, yaw1)
        message_ok = f"rotating {degrees:.1f} deg (CCW positive)"
        message_rej = 'NavigateToPose goal rejected.'
        self._send_goal(goal_msg, message_ok, message_rej)
        resp.success = True
        resp.message = message_ok
        return resp

    @staticmethod
    def _normalize_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    # ---------- Result callback ----------
    def move_base_done_cb(self, fut):
        from action_msgs.msg import GoalStatus
        result = fut.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            message = "Move base action succeeded."
        else:
            message = "Move base action failed."
        self.status_pub.publish(String(data=message))
        self.current_goal_handle = None


def main():
    rclpy.init()
    node = MoveToLocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

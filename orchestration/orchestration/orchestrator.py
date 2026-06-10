#!/usr/bin/env python3
# ROS2 Humble
"""High-level orchestrator / planner / executor for the tetris-assembly task.

Framework choice
----------------
The job per tile is a *fixed linear pipeline*::

    navigate(pickup) -> detect(tile) -> pick(tile) -> navigate(drop) -> drop(tile)

The only reactive element is a human operator who may, on any failure,
**retry** the step or **skip** it (pretend it succeeded and fix it by hand).

Interfaces it drives (already implemented by teammates):
  * navigation : ``move_to`` service (mirte_location_markers/MoveTo). Fire and
                 forget; completion is reported as a String on ``/robot_status``.
  * detection  : publish wanted ids on ``/detection/target_marker_ids``; the node
                 answers with ``/detection/found_marker_id`` + ``/detection/marker_pose``.
  * grasping   : ``pick_tile`` / ``drop_tile`` actions (grasping/PickTile,DropTile),
                 goal is a geometry_msgs/PointStamped.
"""
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PointStamped, PoseStamped

from mirte_location_markers_msgs.srv import MoveTo
from grasping.action import PickTile, DropTile

# Named locations the navigation node understands (move_to(location=...)).
PICK_LOCATION = 'pickup'
DROP_LOCATION = 'drop'

# The mission: for each tile (ArUco id) where to drop it, as (x, y, z) in the
# 'base_link' frame when the robot is at the DROP_LOCATION. Order of this dict = order tiles are processed. The pick pose is
# found live by the detection node, so it is NOT listed here.
TILE_TARGETS: dict[int, tuple] = {
    1: (0.35, 6.75, -0.1),
    2: (0.38, 2.25, -0.1),
    3: (0.38, -2.25, -0.1),
    4: (0.35, -6.75, -0.1),
}

NAV_TIMEOUT = 60.0
DETECT_TIMEOUT = 10.0
GRASP_TIMEOUT = 30.0


class Orchestrator(Node):

    def __init__(self):
        super().__init__('orchestrator')

        # --- navigation -------------------------------------------------
        self.move_cli = self.create_client(MoveTo, 'move_to')
        # move_to is fire-and-forget; the result lands on /robot_status.
        self._nav_done = threading.Event()
        self._nav_ok = False
        self.create_subscription(String, '/robot_status', self._on_status, 10)

        # --- detection --------------------------------------------------
        self.target_pub = self.create_publisher(
            Int32MultiArray, '/detection/target_marker_ids', 10)
        self._want_id = None
        self._last_pose: PoseStamped | None = None
        self._found = threading.Event()
        self.create_subscription(
            PoseStamped, '/detection/marker_pose', self._on_marker_pose, 10)

        # --- grasping ---------------------------------------------------
        self.pick_cli = ActionClient(self, PickTile, 'pick_tile')
        self.drop_cli = ActionClient(self, DropTile, 'drop_tile')

        # --- operator control -------------------------------------------
        self._ctrl_key: str | None = None
        self._ctrl_event = threading.Event()
        self.create_subscription(String, '/orchestrator/control', self._on_control, 10)

    # ==================================================================
    # Subscriber callbacks (run in the executor thread)
    # ==================================================================
    def _on_status(self, msg: String):
        text = msg.data.lower()
        if 'succeeded' in text:
            self._nav_ok = True
            self._nav_done.set()
        elif 'failed' in text or 'rejected' in text:
            self._nav_ok = False
            self._nav_done.set()

    def _on_marker_pose(self, msg: PoseStamped):
        if int(msg.pose.position.z) == self._want_id:
            self._last_pose = msg
            self._found.set()

    def _on_control(self, msg: String):
        key = msg.data.strip().lower()[:1]
        if key in ('r', 's', 'q'):
            self._ctrl_key = key
            self._ctrl_event.set()

    # ==================================================================
    # Phases -- each returns True on success, False on failure.
    # ==================================================================
    def navigate(self, location: str) -> bool:
        if not self.move_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("'move_to' service not available")
            return False
        self._nav_done.clear()
        self._nav_ok = False
        req = MoveTo.Request()
        req.location = location
        fut = self.move_cli.call_async(req)
        if not self._wait(fut, timeout=5.0):
            return False
        resp = fut.result()
        if resp is None or not resp.success:
            self.get_logger().warn(f"move_to({location}) refused: "
                                   f"{getattr(resp, 'message', 'no response')}")
            return False
        # Now wait for Nav2 to actually arrive (reported on /robot_status).
        if not self._nav_done.wait(timeout=NAV_TIMEOUT):
            self.get_logger().warn(f"timed out waiting to reach '{location}'")
            return False
        return self._nav_ok

    def detect(self, tile_id: int) -> PoseStamped | None:
        """Ask detection to look for `tile_id`, return its map-frame pose."""
        self._want_id = tile_id
        self._last_pose = None
        self._found.clear()
        self.target_pub.publish(Int32MultiArray(data=[tile_id]))
        found = self._found.wait(timeout=DETECT_TIMEOUT)
        self.target_pub.publish(Int32MultiArray(data=[]))  # stop searching
        if not found:
            self.get_logger().warn(f"tile {tile_id} not detected")
            return None
        return self._last_pose

    def pick(self, pose: PoseStamped | None) -> bool:
        if pose is None:
            self.get_logger().warn("no pick pose (detection skipped/failed)")
            return False
        pt = PointStamped()
        pt.header = pose.header
        pt.point = pose.pose.position
        return self._run_action(self.pick_cli, PickTile.Goal(pos=pt), 'pick')

    def drop(self, tile_id: int) -> bool:
        x, y, z = TILE_TARGETS[tile_id]
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        # Zero stamp = "use the latest available transform"
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
        return self._run_action(self.drop_cli, DropTile.Goal(pos=pt), 'drop')

    # ==================================================================
    # Mission loop
    # ==================================================================
    def run_mission(self):
        tiles = list(TILE_TARGETS.keys())
        self.get_logger().info(f"Mission: {len(tiles)} tiles -> {tiles}")
        for tile_id in tiles:
            self.get_logger().info(f"===== Tile {tile_id} =====")
            pose: dict = {}
            ok = (
                self._step(f"navigate to '{PICK_LOCATION}'",
                           lambda: self.navigate(PICK_LOCATION))
                and self._step(f"detect tile {tile_id}",
                               lambda: self._detect_into(tile_id, pose))
                and self._step(f"pick tile {tile_id}",
                               lambda: self.pick(pose.get('pose')))
                and self._step(f"navigate to '{DROP_LOCATION}'",
                               lambda: self.navigate(DROP_LOCATION))
                and self._step(f"drop tile {tile_id}",
                               lambda: self.drop(tile_id))
            )
            if not ok:
                self.get_logger().warn("Mission aborted by operator.")
                return
        self.get_logger().info("===== Mission complete =====")

    def _detect_into(self, tile_id: int, store: dict) -> bool:
        store['pose'] = self.detect(tile_id)
        return store['pose'] is not None

    # ==================================================================
    # Helpers
    # ==================================================================
    def _step(self, name: str, fn) -> bool:
        """Run one phase; on failure let the operator retry / skip / quit.

        Returns True to keep going (succeeded or skipped), False to abort.
        """
        while True:
            self.get_logger().info(f"--> {name}")
            try:
                success = fn()
            except Exception as exc:  # keep the mission alive on any glitch
                self.get_logger().error(f"{name} raised: {exc}")
                success = False
            if success:
                self.get_logger().info(f"    OK: {name}")
                return True
            key = self._prompt(name)
            if key == 'r':
                continue
            if key == 's':
                self.get_logger().warn(f"    SKIPPED (assumed done): {name}")
                return True
            return False  # 'q'

    def _prompt(self, name: str) -> str:
        self._ctrl_event.clear()
        self._ctrl_key = None
        self.get_logger().info(
            f"\n[FAILED] {name}\n"
            f"  Publish to /orchestrator/control: 'r' retry  's' skip  'q' quit\n"
            f"  e.g.: ros2 topic pub --once /orchestrator/control std_msgs/String \"data: 'r'\"")
        self._ctrl_event.wait()
        key = self._ctrl_key
        self.get_logger().info(f"Operator chose: {key}")
        return key

    def _run_action(self, client: ActionClient, goal, label: str) -> bool:
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"'{label}' action server not available")
            return False
        send_fut = client.send_goal_async(goal)
        if not self._wait(send_fut, timeout=5.0):
            return False
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{label} goal rejected")
            return False
        result_fut = handle.get_result_async()
        if not self._wait(result_fut, timeout=GRASP_TIMEOUT):
            self.get_logger().warn(f"{label} timed out")
            return False
        result = result_fut.result()
        if result is None or not result.result.success:
            msg = getattr(getattr(result, 'result', None), 'message', '')
            self.get_logger().warn(f"{label} failed: {msg}")
            return False
        return True

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        """Block until `future` is done. The executor (other thread) services it."""
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline or not rclpy.ok():
                return False
            time.sleep(0.02)
        return True



def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()

    # Spin in a background thread so the mission loop can block on results
    # and read the keyboard in the main thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

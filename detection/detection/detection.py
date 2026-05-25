import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('nth_image_detection')

        self.declare_parameter('image_topic', '/gripper_camera/image_raw/compressed')
        self.declare_parameter('process_every_n', 3)

        self._image_topic = self.get_parameter('image_topic').value
        self._process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self._frame_count = 0

        # Keep the queue short so we always work on fresh frames.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            qos,
        )

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._process_every_n != 0:
            return

        # TODO: Run detection


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

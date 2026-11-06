#!/usr/bin/env python3

import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DualFisheyeCombiner(Node):
    """Join the independently decoded front and back sensor images."""

    def __init__(self):
        super().__init__("dual_fisheye_combiner")

        self.declare_parameter("front_topic", "/dual_fisheye/front/image")
        self.declare_parameter("back_topic", "/dual_fisheye/back/image")
        self.declare_parameter("output_topic", "/dual_fisheye/image")

        front_topic = self.get_parameter("front_topic").value
        back_topic = self.get_parameter("back_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.front_image = None
        self.back_image = None
        self.front_subscription = self.create_subscription(
            Image, front_topic, self.front_callback, 10
        )
        self.back_subscription = self.create_subscription(
            Image, back_topic, self.back_callback, 10
        )
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f"Combining {back_topic} (left) + {front_topic} (right) -> {output_topic}"
        )

    def front_callback(self, msg):
        self._update("front", msg)

    def back_callback(self, msg):
        self._update("back", msg)

    def _update(self, side, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert {side} image: {exc}")
            return

        with self.lock:
            if side == "front":
                self.front_image = image.copy()
            else:
                self.back_image = image.copy()

            if self.front_image is None or self.back_image is None:
                return

            front = self.front_image
            back = self.back_image

        if front.shape[0] != back.shape[0]:
            self.get_logger().warning(
                f"Sensor heights differ: front={front.shape}, back={back.shape}; resizing back"
            )
            back = cv2.resize(back, (back.shape[1], front.shape[0]))

        combined = cv2.hconcat([back, front])
        output = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        output.header = msg.header
        output.header.frame_id = "camera_dual_fisheye"
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = DualFisheyeCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

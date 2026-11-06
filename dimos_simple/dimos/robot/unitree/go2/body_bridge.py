"""Go2 body data and explicit ROS control; no navigation or camera forwarding."""

import json
import os
from threading import Thread

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.wired_link import WiredLink


class Go2BodyBridge(GO2Connection):
    """Reuse the native driver without GO2Connection's automatic posture changes.

    The underlying driver still selects its normal motion mode during connection,
    and sends a zero joystick command on disconnect. Nonzero commands originate
    only from /cmd_vel or an explicit RPC call.
    """

    def _make_connection(self) -> UnitreeWebRTCConnection:
        link = WiredLink.check(self.config.ip)
        return UnitreeWebRTCConnection(self.config.ip, aes_128_key=self.config.aes_128_key, wired_link=link)

    @rpc
    def start(self) -> None:
        Module.start(self)
        self._start_ros()
        if hasattr(self.connection, "set_ros_clock"):
            self.connection.set_ros_clock(self._body_node.get_clock())
        self._odom_trace = None
        if os.environ.get("DIMOS_ODOM_TRACE"):
            from dimos.robot.unitree.go2.odom_trace import install_trace
            self._odom_trace = install_trace(self, os.environ["DIMOS_ODOM_TRACE"])
        self.connection.start()
        self._disposables.add(self.connection.odom_stream().subscribe(self._publish_tf))
        self._disposables.add(self.connection.lidar_stream().subscribe(self.lidar.publish))
        if hasattr(self.connection, "lowstate_stream"):
            self._disposables.add(
                self.connection.lowstate_stream().subscribe(self._publish_lowstate)
            )
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

    def _start_ros(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage

        if not rclpy.ok():
            rclpy.init()
        self._body_node = Node("go2_body_bridge")
        self._body_executor = SingleThreadedExecutor()
        self._body_executor.add_node(self._body_node)
        self._tf_publisher = self._body_node.create_publisher(TFMessage, "/tf", 10)
        self._lowstate_publisher = self._body_node.create_publisher(
            String, "/go2/lowstate", qos_profile_sensor_data
        )
        self._body_services = [
            self._body_node.create_service(
                Trigger, f"/go2/{name}", self._control_callback(action)
            )
            for name, action in (
                ("standup", self.connection.standup),
                ("liedown", self.connection.liedown),
                ("balance_stand", self.connection.balance_stand),
            )
        ]
        self._body_node.create_timer(1.0, self._publish_calibration)
        self._body_thread = Thread(target=self._body_executor.spin, daemon=True)
        self._body_thread.start()

    @staticmethod
    def _control_callback(action):
        def callback(request, response):
            try:
                response.success = bool(action())
                response.message = "completed" if response.success else "robot rejected command"
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response
        return callback

    def _publish_calibration(self) -> None:
        if self._realsense_camera_info_static is not None:
            self.realsense_camera_info.publish(
                self._realsense_camera_info_static.with_ts(
                    self._body_node.get_clock().now().nanoseconds / 1_000_000_000
                )
            )

    def _publish_lowstate(self, msg) -> None:
        from std_msgs.msg import String

        self._lowstate_publisher.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def _publish_tf(self, msg: PoseStamped) -> None:
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        # Odom is independent of TF serialization/publication.
        self.odom.publish(msg)
        transforms = []
        for transform in self._odom_to_tf(msg):
            stamped = TransformStamped()
            nanoseconds = round(transform.ts * 1_000_000_000)
            stamped.header.stamp.sec, stamped.header.stamp.nanosec = divmod(
                nanoseconds, 1_000_000_000
            )
            stamped.header.frame_id = transform.frame_id
            stamped.child_frame_id = transform.child_frame_id
            for axis in ("x", "y", "z"):
                setattr(stamped.transform.translation, axis, float(getattr(transform.translation, axis)))
            for axis in ("x", "y", "z", "w"):
                setattr(stamped.transform.rotation, axis, float(getattr(transform.rotation, axis)))
            transforms.append(stamped)
        self._tf_publisher.publish(TFMessage(transforms=transforms))

    @rpc
    def stop(self) -> None:
        # Unsubscribe before closing publishers. Never call GO2Connection.stop(),
        # which commands LieDown; the native disconnect retains its stop command.
        self._disposables.dispose()
        trace = getattr(self, "_odom_trace", None)
        if trace is not None:
            trace.stop()
        try:
            self.connection.stop()
        finally:
            executor = getattr(self, "_body_executor", None)
            if executor is not None:
                executor.shutdown(timeout_sec=2.0)
                self._body_thread.join(timeout=2.0)
                self._body_node.destroy_node()
                self._body_executor = None
            Module.stop(self)

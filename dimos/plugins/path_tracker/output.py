"""Velocity output boundary: preview never constructs a ROS command publisher."""

import json


class VelocityOutput:
    def __init__(self, node, topic, output_only):
        self.publisher = None
        self.message_type = None
        if not output_only:
            from geometry_msgs.msg import Twist

            self.message_type = Twist
            self.publisher = node.create_publisher(Twist, topic, 10)

    def __call__(self, v, w, reason):
        if self.publisher is None:
            print(
                json.dumps(
                    {
                        "linear_x": v,
                        "angular_z": w,
                        "reason": reason,
                        "published": False,
                    }
                ),
                flush=True,
            )
            return False
        msg = self.message_type()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.publisher.publish(msg)
        return True

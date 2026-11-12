"""Publish a read-only view of the active exploration task for RViz."""

import math


class ExplorationVisualization:
    def __init__(self, node):
        from geometry_msgs.msg import Point
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import Marker, MarkerArray

        self.node = node
        self.Point = Point
        self.OccupancyGrid = OccupancyGrid
        self.Marker = Marker
        self.MarkerArray = MarkerArray
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_publisher = node.create_publisher(OccupancyGrid, "/vln/exploration/map", qos)
        try:
            self.marker_publisher = node.create_publisher(
                MarkerArray, "/vln/exploration/markers", qos,
            )
        except BaseException:
            node.destroy_publisher(self.map_publisher)
            raise
        self.last_grid = None

    def _grid_message(self, snapshot):
        message = self.OccupancyGrid()
        message.header.frame_id = snapshot.frame
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            int(snapshot.stamp_ns), 1_000_000_000,
        )
        message.info.resolution = float(snapshot.resolution)
        message.info.height, message.info.width = snapshot.grid.shape
        origin = message.info.origin
        origin.position.x, origin.position.y = map(float, snapshot.origin_xy)
        origin.orientation.w = 1.0
        return message

    def _marker(self, header, namespace, marker_type, scale, color, *, point=None, yaw=0.0, index=0):
        marker = self.Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = index
        marker.type = marker_type
        marker.action = self.Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.orientation.z = math.sin(yaw / 2)
        marker.pose.orientation.w = math.cos(yaw / 2)
        if point is not None:
            marker.pose.position.x, marker.pose.position.y = map(float, point[:2])
            marker.pose.position.z = 0.06
        return marker

    def _delete_markers(self, header):
        marker = self.Marker()
        marker.header = header
        marker.action = self.Marker.DELETEALL
        return marker

    def update(self, grid, pose, trail, *, goal=None, cooling=()):
        message = self._grid_message(grid)
        message.data = grid.grid.ravel().tolist()
        header = message.header
        marker_type = self.Marker
        markers = [self._delete_markers(header)]
        for index, observed in enumerate(trail):
            markers.append(self._marker(
                header, "observed", marker_type.ARROW, (0.22, 0.03, 0.03),
                (0.27, 0.61, 0.99, 0.85), point=observed, yaw=observed[2], index=index,
            ))
        markers.append(self._marker(
            header, "robot", marker_type.ARROW, (0.35, 0.06, 0.06),
            (0.20, 0.90, 0.45, 1.0), point=pose, yaw=pose[2],
        ))
        if goal is not None:
            markers.append(self._marker(
                header, "frontier", marker_type.SPHERE, (0.24, 0.24, 0.24),
                (0.97, 0.68, 0.20, 1.0), point=goal.frontier_xy,
            ))
            markers.append(self._marker(
                header, "waypoint", marker_type.CUBE, (0.13, 0.13, 0.13),
                (0.20, 0.92, 0.85, 1.0), point=goal.xy,
            ))
        cooling_points = [
            self.Point(x=float(x), y=float(y), z=0.06) for x, y in cooling
        ]
        if cooling_points:
            failed = self._marker(
                header, "cooldown", marker_type.SPHERE_LIST, (0.15, 0.15, 0.15),
                (0.94, 0.31, 0.27, 0.85),
            )
            failed.points = cooling_points
            markers.append(failed)
        self.last_grid = grid
        self.map_publisher.publish(message)
        self.marker_publisher.publish(self.MarkerArray(markers=markers))

    def clear(self):
        if self.last_grid is None:
            return
        message = self._grid_message(self.last_grid)
        message.data = [-1] * self.last_grid.grid.size
        self.map_publisher.publish(message)
        self.marker_publisher.publish(self.MarkerArray(
            markers=[self._delete_markers(message.header)],
        ))
        self.last_grid = None

    def close(self):
        try:
            self.clear()
        finally:
            self.node.destroy_publisher(self.marker_publisher)
            self.node.destroy_publisher(self.map_publisher)

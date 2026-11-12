from types import SimpleNamespace

import numpy as np
import pytest

from dimos.vln_task.search.frontier import FrontierGoal
from dimos.vln_task.search.occupancy import GridSnapshot


class RecordingNode:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, message_type, topic, qos):
        messages = []
        publisher = SimpleNamespace(publish=messages.append, messages=messages, qos=qos)
        self.publishers[topic] = publisher
        return publisher

    def destroy_publisher(self, publisher):
        topic = next(name for name, value in self.publishers.items() if value is publisher)
        del self.publishers[topic]


@pytest.fixture
def display():
    pytest.importorskip("nav_msgs.msg")
    pytest.importorskip("visualization_msgs.msg")
    from dimos.vln_task.search.visualization import ExplorationVisualization

    node = RecordingNode()
    snapshot = GridSnapshot(
        np.array([[-1, 0, 100], [0, 0, -1]], dtype=np.int8),
        (3.0, 4.0), 0.1, "world", 9_000_000_032,
    )
    return ExplorationVisualization(node), node, snapshot


def test_display_publishes_source_grid_and_distinct_goal_markers(display):
    from rclpy.qos import DurabilityPolicy

    visualization, node, snapshot = display
    visualization.update(
        snapshot, (3.5, 4.0, 0.0), ((3.0, 4.0, 0.0),),
        goal=FrontierGoal((4.0, 4.0), (4.5, 4.5), (), False),
        cooling=((4.5, 5.0),),
    )

    grid_publisher = node.publishers["/vln/exploration/map"]
    assert grid_publisher.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert grid_publisher.qos.depth == 1
    grid = grid_publisher.messages[-1]
    assert grid.header.frame_id == "world"
    assert (grid.header.stamp.sec, grid.header.stamp.nanosec) == (9, 32)
    assert (grid.info.height, grid.info.width) == (2, 3)
    assert (grid.info.origin.position.x, grid.info.origin.position.y) == (3.0, 4.0)
    assert grid.info.resolution == pytest.approx(0.1)
    assert list(grid.data) == [-1, 0, 100, 0, 0, -1]

    marker_publisher = node.publishers["/vln/exploration/markers"]
    assert marker_publisher.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert marker_publisher.qos.depth == 1
    markers = {marker.ns: marker for marker in marker_publisher.messages[-1].markers}
    assert markers["frontier"].pose.position.x == 4.5
    assert markers["waypoint"].pose.position.x == 4.0
    assert markers["cooldown"].points[0].y == 5.0
    assert markers["observed"].pose.position.x == 3.0


def test_clear_replaces_latched_task_data_and_close_destroys_publishers(display):
    from visualization_msgs.msg import Marker

    visualization, node, snapshot = display
    visualization.update(snapshot, (3.5, 4.0, 0.0), ())

    visualization.clear()

    grid = node.publishers["/vln/exploration/map"].messages[-1]
    assert list(grid.data) == [-1] * 6
    markers = node.publishers["/vln/exploration/markers"].messages[-1].markers
    assert [marker.action for marker in markers] == [Marker.DELETEALL]
    visualization.close()
    assert not node.publishers

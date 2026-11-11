# compressed_RGBD_pkg

C++ RGB-D compressor for the D455 edge camera.

The node preserves the original ROS topic contract:

- `/camera/camera/color/image_raw` -> `/camera/camera/color/image_raw/compressed` (`jpeg`)
- `/camera/camera/aligned_depth_to_color/image_raw` ->
  `/camera/camera/aligned_depth_to_color/image_raw/compressed`
  (`16UC1; compressedDepth png`)

Each stream has three independent stages:

1. The ROS callback stores only the newest raw frame.
2. A dedicated encoder thread converts the image and runs JPEG/PNG encoding.
3. A dedicated publisher thread calls `publish()`, so a DDS send stall cannot
   block frame reception or encoding.

`config/compressed_rgbd_dds.xml` gives this process UDPv4 + SHM transports and
an asynchronous Fast DDS writer backed by the `compressed_async` flow
controller. TCP is intentionally absent from the image process. Compressed
images use BEST_EFFORT with a one-frame history so queued or retransmitted
frames cannot delay the next fresh image.

The launch sets `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` and
`RMW_FASTRTPS_USE_QOS_FROM_XML=1` so rmw_fastrtps creates the writer with the
XML publish mode and flow controller.

The node writes one small atomic telemetry record per second to
`log/camera_state/domain_<ROS_DOMAIN_ID>/compressed_d455_metrics.json`. It
contains the raw RGB/depth receive rates, compressed RGB/depth publish rates,
frame ages, and cumulative queue-drop counters. The camera dashboard reads this
record without subscribing to image topics, so monitoring does not add another
large-frame DDS consumer.

RGB and depth messages are stamped once when they enter this local ROS 2 node.
The compressed messages carry that local ROS-clock timestamp so RGB, depth and
the locally published odometry can be correlated on one time base by downstream
services. Consumers must read and preserve `header.stamp`; they must not replace
it with decode or network-arrival time.

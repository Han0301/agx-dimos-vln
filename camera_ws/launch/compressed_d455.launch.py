"""Start only the C++ D455 RGB-D compression node."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _metrics_path():
    explicit = os.environ.get('CAMERA_STATE_DIR')
    if explicit:
        directory = Path(explicit).expanduser()
    else:
        workspace = Path(__file__).resolve().parents[1]
        base = Path(os.environ.get('ROS_LOG_DIR', workspace / 'log')).expanduser()
        directory = base / 'camera_state' / ('domain_' + os.environ.get('ROS_DOMAIN_ID', '0'))
    return str(directory / 'compressed_d455_metrics.json')


def generate_launch_description():
    defaults = {
        'color_quality': '80',
        'depth_compression_level': '1',
        'stats_period_s': '10.0',
        'trace_slow_ms': '0.0',
    }
    executable = PathJoinSubstitution([
        FindPackagePrefix('compressed_RGBD_pkg'),
        'lib', 'compressed_RGBD_pkg', 'compressed_rgbd_node',
    ])
    dds_config = PathJoinSubstitution([
        FindPackageShare('compressed_RGBD_pkg'),
        'config', 'compressed_rgbd_dds.xml',
    ])
    compressor = ExecuteProcess(
        name='compressed_rgbd_node',
        output='screen',
        cmd=[
            executable,
            '--ros-args',
            '-p', ['color_quality:=', LaunchConfiguration('color_quality')],
            '-p', ['depth_compression_level:=', LaunchConfiguration('depth_compression_level')],
            '-p', ['stats_period_s:=', LaunchConfiguration('stats_period_s')],
            '-p', ['trace_slow_ms:=', LaunchConfiguration('trace_slow_ms')],
            '-p', 'metrics_path:=' + _metrics_path(),
        ],
        additional_env={
            'FASTRTPS_DEFAULT_PROFILES_FILE': dds_config,
            'FASTDDS_BUILTIN_TRANSPORTS': 'NONE',
            'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS',
            'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        },
    )
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()
        ],
        compressor,
    ])

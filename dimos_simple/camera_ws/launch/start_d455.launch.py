"""Start only the raw D455 color and aligned-depth streams."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from insta360_ros_driver.camera_state import begin_device, record_error, stop_device
from insta360_ros_driver.console_status import CameraDiagnostics, status_line


def _abort(message, diagnostics):
    detail = message
    if diagnostics.summary():
        detail += ' Key cause: ' + diagnostics.summary()
    if diagnostics.run_id:
        record_error('D455', diagnostics.run_id, detail, fatal=True)
    raise RuntimeError(detail)


def _setup(context):
    run_id = begin_device('D455', '正在启动 D455 原始彩色和对齐深度。')
    diagnostics = CameraDiagnostics('D455', ['realsense', 'wait_d455', 'd455_'], run_id=run_id)
    helper = PathJoinSubstitution([
        FindPackagePrefix('insta360_ros_driver'),
        'lib', 'insta360_ros_driver', 'wait_for_camera_data.py',
    ])
    python = LaunchConfiguration('python')
    topics = [
        '/camera/camera/color/image_raw',
        '/camera/camera/aligned_depth_to_color/image_raw',
    ]

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
        ])),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
        }.items(),
    )
    first_frames = ExecuteProcess(
        name='wait_d455_images',
        output='screen',
        cmd=[
            python, '-u', helper, '--topics', *topics,
            '--timeout', LaunchConfiguration('startup_timeout_sec'),
        ],
    )
    watchdog = ExecuteProcess(
        name='d455_stream_watchdog',
        output='screen',
        cmd=[
            python, '-u', helper, '--topics', *topics, '--watchdog',
            '--state-group', 'D455',
            '--timeout', LaunchConfiguration('stream_timeout_sec'),
        ],
        additional_env={'CAMERA_STATE_RUN_ID': run_id},
    )

    def after_images(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            return [OpaqueFunction(
                function=lambda _: _abort(
                    'D455 color/aligned-depth frames not ready.', diagnostics)),
            ]
        return [
            LogInfo(msg=status_line(
                'D455', 'OK', 'D455 raw color and aligned depth are ready.')),
            watchdog,
        ]

    def stop_on_exit(event, launch_context, label):
        if launch_context.is_shutdown:
            return []
        return [OpaqueFunction(
            function=lambda _: _abort(
                f'{label} stopped (code {event.returncode}).', diagnostics)),
        ]

    return [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda _event, _context: stop_device('D455', run_id))),
        RegisterEventHandler(OnProcessIO(
            on_stdout=diagnostics.feed, on_stderr=diagnostics.feed)),
        RegisterEventHandler(OnProcessExit(
            target_action=first_frames, on_exit=after_images)),
        RegisterEventHandler(OnProcessExit(
            target_action=watchdog,
            on_exit=lambda event, launch_context: stop_on_exit(
                event, launch_context, 'D455 stream watchdog'))),
        LogInfo(msg=status_line('D455', 'WAIT', 'Starting D455.')),
        realsense,
        first_frames,
    ]


def generate_launch_description():
    defaults = {
        'color_profile': '848,480,30',
        'depth_profile': '424,240,30',
        'python': '/usr/bin/python3',
        'startup_timeout_sec': '45',
        'stream_timeout_sec': '10',
    }
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()
        ],
        OpaqueFunction(function=_setup),
    ])

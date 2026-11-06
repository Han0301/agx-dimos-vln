"""Start RealSense, wait for color AND aligned depth, then start compresser.py."""
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.logging import get_logger
from insta360_ros_driver.console_status import CameraDiagnostics, status_line
from insta360_ros_driver.camera_state import begin_device, record_error, stop_device

# Keep the same failure handling as the X3 startup sequence.


def abort(context, message, diagnostics=None):
    if diagnostics is not None and diagnostics.summary():
        message += ' Key cause: ' + diagnostics.summary()
    if diagnostics is not None and diagnostics.run_id:
        record_error(diagnostics.group, diagnostics.run_id, message, fatal=True)
    get_logger('camera.state').error(status_line('D455', 'ERROR', message))
    raise RuntimeError(message)


def stop_on_exit(event, context, label, diagnostics=None):
    if context.is_shutdown:
        return []
    return [OpaqueFunction(function=abort, args=[f'{label} stopped (code {event.returncode}); stopping D455 group.', diagnostics])]


def mark_stopped(event, context, run_id):
    stop_device('D455', run_id)


def setup(context):
    run_id = begin_device('D455', '正在启动并等待相机真实数据。')
    diagnostics = CameraDiagnostics('D455', ['realsense', 'wait_d455', 'compresser', 'd455_'], run_id=run_id)
    script = Path(LaunchConfiguration('compresser_script').perform(context)).expanduser().resolve()
    if not script.is_file():
        abort(context, f'Compression script not found: {script}')
    helper = PathJoinSubstitution([FindPackagePrefix('insta360_ros_driver'), 'lib', 'insta360_ros_driver', 'wait_for_camera_data.py'])
    python = LaunchConfiguration('python')
    topics = ['/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw']
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_namespace': 'camera', 'camera_name': 'camera',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
        }.items())
    first_frames = ExecuteProcess(name='wait_d455_images', output='screen', cmd=[
        python, '-u', helper, '--topics', *topics, '--timeout', LaunchConfiguration('startup_timeout_sec')])
    compressor = ExecuteProcess(
        name='compressed_rgbd_node',
        output='screen',
        cmd=[PathJoinSubstitution([
            FindPackagePrefix('compressed_RGBD_pkg'),
            'lib', 'compressed_RGBD_pkg', 'compressed_rgbd_node',
        ])],
        additional_env={
            'FASTRTPS_DEFAULT_PROFILES_FILE': PathJoinSubstitution([
                FindPackageShare('compressed_RGBD_pkg'),
                'config', 'compressed_rgbd_dds.xml',
            ]),
            'FASTDDS_BUILTIN_TRANSPORTS': 'NONE',
            'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS',
            'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        },
    )
    watchdog = ExecuteProcess(name='d455_stream_watchdog', output='screen', cmd=[
        python, '-u', helper, '--topics', *topics, '--watchdog', '--state-group', 'D455',
        '--timeout', LaunchConfiguration('stream_timeout_sec')],
        additional_env={'CAMERA_STATE_RUN_ID': run_id})

    def after_images(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [OpaqueFunction(function=abort, args=['D455 color/aligned-depth frames not ready; compressor will not start.', diagnostics])]
        return [LogInfo(msg=status_line('D455', 'OK', 'D455 color and aligned depth ready; starting compresser.py.')), compressor, watchdog]

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: mark_stopped(event, context, run_id))),
        RegisterEventHandler(OnProcessIO(on_stdout=diagnostics.feed, on_stderr=diagnostics.feed)),
        RegisterEventHandler(OnProcessExit(target_action=first_frames, on_exit=after_images)),
        RegisterEventHandler(OnProcessExit(target_action=compressor,
            on_exit=lambda event, context: stop_on_exit(event, context, 'C++ RGB-D compressor', diagnostics))),
        RegisterEventHandler(OnProcessExit(target_action=watchdog,
            on_exit=lambda event, context: stop_on_exit(event, context, 'D455 image watchdog', diagnostics))),
        LogInfo(msg=status_line('D455', 'WAIT', 'Waiting for real color and aligned-depth frames.')),
        realsense, first_frames,
    ]


def generate_launch_description():
    defaults = {
        'color_profile': '848,480,30', 'depth_profile': '424,240,30',
        'compresser_script': str(Path(get_package_prefix('insta360_ros_driver')) / 'lib/insta360_ros_driver/compresser.py'),
        'python': '/usr/bin/python3', 'startup_timeout_sec': '45', 'stream_timeout_sec': '10',
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=setup)])

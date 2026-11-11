"""USB -> SDK streaming -> processing; stop the whole group on camera failure."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.logging import get_logger
from insta360_ros_driver.console_status import CameraDiagnostics, status_line
from insta360_ros_driver.camera_state import begin_device, record_error, stop_device


def abort(context, message, diagnostics=None):
    if diagnostics is not None and diagnostics.summary():
        message += ' Key cause: ' + diagnostics.summary()
    if diagnostics is not None and diagnostics.run_id:
        record_error(diagnostics.group, diagnostics.run_id, message, fatal=True)
    get_logger('camera.state').error(status_line('X3', 'ERROR', message))
    raise RuntimeError(message)


def after_gate(event, context, actions, label, diagnostics=None):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        return [OpaqueFunction(function=abort, args=[f'{label} failed; stopping this camera group.', diagnostics])]
    return actions


def fail_on_exit(event, context, label, diagnostics=None):
    if context.is_shutdown:
        return []
    return [OpaqueFunction(function=abort, args=[f'{label} exited (code {event.returncode}); stopping this camera group.', diagnostics])]


def mark_stopped(event, context, run_id):
    stop_device('X3', run_id)


def setup(context):
    run_id = begin_device('X3', '正在启动并等待相机真实数据。')
    diagnostics = CameraDiagnostics('X3', ['x3_', 'insta360_ros_driver', 'decoder', 'equirectangular_cpp', 'surround_views', 'imu_filter'], run_id=run_id)
    share = FindPackageShare('insta360_ros_driver')
    helper = PathJoinSubstitution([FindPackagePrefix('insta360_ros_driver'), 'lib', 'insta360_ros_driver', 'wait_for_camera_data.py'])
    python = LaunchConfiguration('python')
    usb = ExecuteProcess(name='x3_usb_check', output='screen', cmd=[
        python, '-u', helper, '--usb-vendor', '2e1a', '--timeout', LaunchConfiguration('usb_timeout_sec')])
    driver = Node(package='insta360_ros_driver', executable='insta360_ros_driver',
                  name='insta360_ros_driver', output='screen', cwd=PathJoinSubstitution([share, 'lib']))
    first_frame = ExecuteProcess(name='x3_first_frame', output='screen', cmd=[
        python, '-u', helper, '--compressed-topics', '/dual_fisheye/image/compressed',
        '--timeout', LaunchConfiguration('startup_timeout_sec')])
    watchdog = ExecuteProcess(name='x3_stream_watchdog', output='screen', cmd=[
        python, '-u', helper, '--compressed-topics', '/dual_fisheye/image/compressed',
        '--watchdog', '--state-group', 'X3', '--timeout', LaunchConfiguration('stream_timeout_sec')],
        additional_env={'CAMERA_STATE_RUN_ID': run_id})
    processing = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'processing.launch.xml'])),
        launch_arguments={name: LaunchConfiguration(name) for name in [
            'equirectangular', 'surround', 'decoder', 'hardware_decoder', 'imu_filter',
            'equirectangular_config', 'imu_config']}.items())
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: mark_stopped(event, context, run_id))),
        RegisterEventHandler(OnProcessIO(on_stdout=diagnostics.feed, on_stderr=diagnostics.feed)),
        RegisterEventHandler(OnProcessExit(target_action=usb,
            on_exit=lambda event, context: after_gate(event, context, [driver, first_frame], 'X3 USB check', diagnostics))),
        RegisterEventHandler(OnProcessExit(target_action=driver,
            on_exit=lambda event, context: fail_on_exit(event, context, 'X3 driver', diagnostics))),
        RegisterEventHandler(OnProcessExit(target_action=first_frame,
            on_exit=lambda event, context: after_gate(event, context,
                [LogInfo(msg=status_line('X3', 'OK', 'X3 is streaming; starting decoder, panorama and selected processing nodes.')), processing, watchdog],
                'X3 first frame', diagnostics))),
        RegisterEventHandler(OnProcessExit(target_action=watchdog,
            on_exit=lambda event, context: fail_on_exit(event, context, 'X3 stream watchdog', diagnostics))),
        LogInfo(msg=status_line('X3', 'WAIT', 'Waiting for USB access and a real camera stream.')),
        usb,
    ]


def generate_launch_description():
    share = FindPackageShare('insta360_ros_driver')
    defaults = {
        'equirectangular': 'false', 'surround': 'false', 'decoder': 'true',
        'hardware_decoder': 'false', 'imu_filter': 'true',
        'equirectangular_config': PathJoinSubstitution([share, 'config', 'equirectangular.yaml']),
        'imu_config': PathJoinSubstitution([share, 'config', 'imu_filter.yaml']),
        'usb_timeout_sec': '30', 'startup_timeout_sec': '30', 'stream_timeout_sec': '10',
        'python': '/usr/bin/python3',
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=setup)])

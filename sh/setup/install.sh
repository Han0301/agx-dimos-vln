#!/usr/bin/env bash
# One entry point for installation and repeatable environment verification.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
WS="$ROOT/camera_ws"
VENV="$ROOT/.venv"
PYTHON=/usr/bin/python3.12
camera=d455
plugin=all
check_only=0 dry_run=0 recreate=0 install_system=1 install_system_requested=0 skip_build=0 offline=0 sdk_requested=0
sdk_dir="${CAMERA_SDK_DIR:-}"
sdk_archive="${CAMERA_SDK_ARCHIVE:-}"
sdk_url="${CAMERA_SDK_URL:-}"
sdk_sha256="${CAMERA_SDK_SHA256:-}"
temp=''
cleanup() { [[ -z "$temp" ]] || rm -rf -- "$temp"; }
trap cleanup EXIT
trap 'printf "ERROR: stopped at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
usage() {
  cat <<'HELP'
Usage: bash sh/setup/install.sh [options]
  --plugin none|path_tracker|agent|all  Local dependencies (default: all)
  --camera d455|x3|all|none  Cameras to prepare/check (default: d455)
  --sdk-dir PATH           Extracted Insta360 CameraSDK (X3/all)
  --sdk-archive PATH       Official CameraSDK ZIP/TAR archive (X3/all)
  --sdk-url URL            Download CameraSDK from a supplied URL (X3/all)
  --sdk-sha256 HEX         Verify the SDK archive/download
  --install-system-deps    Install missing Ubuntu packages via apt/sudo (already default)
  --recreate               Force backup/rebuild of .venv; incompatible venv is rebuilt automatically
  --skip-build             Reuse camera build outputs; still check them
  --offline                Use Python cache; forbid SDK URL downloads
  --check-only             Only check existing software; no install/build
  --dry-run                Print installation steps without executing them
  -h, --help               Show help

No arguments: prepare D455 + C++ compressor + Go2 + XNavDP client + tracker + standalone agent clients.
Install missing system dependencies (sudo may prompt), back up incompatible venv,
install into this workspace .venv, incrementally build cameras and verify.
Re-run the same command after installation to repair dependencies and check.
Target: Ubuntu 24.04 ARM64, Python 3.12, bundled camera_ws/vendor ROS Jazzy.
Restore a matching camera_ws runtime bundle if vendor/ is missing.
USB permissions, device data and Go2 connectivity are reported separately.
This command never starts cameras, connects to Go2 or sends motion commands.
HELP
}
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
info() { printf '\n== %s ==\n' "$*"; }
run() {
  if ((dry_run)); then printf '  '; printf '%q ' "$@"; printf '\n'; else "$@"; fi
}
while (($#)); do
  case "$1" in
    --plugin) plugin="${2:?missing plugin selection}"; shift 2;;
    --camera) camera="${2:?missing camera selection}"; shift 2;;
    --sdk-dir) sdk_requested=1; sdk_dir="${2:?missing SDK directory}"; shift 2;;
    --sdk-archive) sdk_requested=1; sdk_archive="${2:?missing SDK archive}"; shift 2;;
    --sdk-url) sdk_requested=1; sdk_url="${2:?missing SDK URL}"; shift 2;;
    --sdk-sha256) sdk_requested=1; sdk_sha256="${2:?missing SHA256}"; shift 2;;
    --install-system-deps) install_system_requested=1; shift;;
    --recreate) recreate=1; shift;;
    --skip-build) skip_build=1; shift;;
    --offline) offline=1; shift;;
    --check-only) check_only=1; shift;;
    --dry-run) dry_run=1; shift;;
    -h|--help) usage; exit 0;;
    *) die "unknown option: $1 (see --help)";;
  esac
done
case "$plugin" in none|path_tracker|agent|all);; *) die "invalid --plugin: $plugin";; esac
case "$camera" in d455|x3|all|none);; *) die "invalid --camera: $camera";; esac
if ((sdk_requested)) && [[ "$camera" != x3 && "$camera" != all ]]; then
  die 'SDK options require --camera x3 or --camera all'
fi
((check_only == 0 || sdk_requested == 0)) || die '--check-only does not install SDK sources'
((check_only == 0 || (recreate == 0 && install_system_requested == 0 && dry_run == 0))) ||
  die '--check-only conflicts with --recreate, --install-system-deps and --dry-run'
[[ -z "$sdk_sha256" || "$sdk_sha256" =~ ^[[:xdigit:]]{64}$ ]] || die 'invalid SDK SHA256'
((offline == 0)) || [[ -z "$sdk_url" ]] || die '--offline conflicts with SDK URL'

preflight() {
  info 'Workspace prerequisites'
  [[ -f "$ROOT/dimos/pyproject.toml" && -f "$ROOT/dimos/requirements.lock" ]] || die 'incomplete DimOS source'
  if ((dry_run)); then
    printf 'Target: Ubuntu 24.04 aarch64 / Python 3.12; runtime: %s/vendor\n' "$WS"
    return
  fi
  [[ "$(uname -m)" == aarch64 ]] || die 'bundled runtime requires aarch64'
  local ID VERSION_ID
  . /etc/os-release
  [[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || die 'bundled runtime requires Ubuntu 24.04'
  [[ -d "$WS/vendor/ros/jazzy/lib/python3.12/site-packages" &&
     -d "$WS/vendor/compat/opt/ros/jazzy" ]] ||
    die 'ROS runtime missing: restore a matching camera_ws deployment bundle; see camera_ws/README.md'
}
system_dependencies() {
  info 'System dependencies'
  local -a packages=(python3.12-venv python3.12-dev build-essential pkg-config ca-certificates
    libportaudio2 portaudio19-dev libasound2-dev libgl1 libegl1)
  if [[ "$camera" != none ]]; then
    packages+=(cmake python3-colcon-common-extensions python3-setuptools python3-catkin-pkg
      python3-empy python3-lark python3-yaml python3-numpy python3-opencv
      libusb-1.0-0-dev libudev-dev libopencv-dev libssl-dev usbutils udev)
  fi
  if [[ "$camera" == x3 || "$camera" == all ]]; then
    packages+=(curl unzip ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev)
  fi
  local package
  local -a missing=() sudo_cmd=()
  for package in "${packages[@]}"; do
    if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != 'install ok installed' ]]; then
      missing+=("$package")
    fi
  done
  if ((${#missing[@]})); then
    if ((install_system && !check_only && !offline)); then
      ((EUID == 0)) || sudo_cmd=(sudo)
      run "${sudo_cmd[@]}" apt-get -o Acquire::Retries=1 -o Acquire::http::Timeout=15 -o Acquire::https::Timeout=15 update
      run "${sudo_cmd[@]}" apt-get install -y "${missing[@]}"
    elif ((dry_run)); then
      printf 'Missing system packages: %s\n' "${missing[*]}"
    else
      die "missing system packages: ${missing[*]}; rerun bash sh/setup/install.sh"
    fi
  fi
}
validate_venv() {
  [[ ! -L "$VENV" && -x "$VENV/bin/python" ]] || die 'invalid/symlinked .venv; run sh/setup/install.sh'
  env -u PYTHONPATH -u PYTHONHOME "$VENV/bin/python" - "$VENV" <<'PY'
import pathlib, sys
root = pathlib.Path(sys.argv[1]).resolve()
if sys.version_info[:2] != (3, 12) or pathlib.Path(sys.prefix).resolve() != root:
    raise SystemExit('Wrong venv Python/prefix; run sh/setup/install.sh')
if 'include-system-site-packages = true' in (root / 'pyvenv.cfg').read_text().lower():
    raise SystemExit('Existing venv inherits system packages; rerun sh/setup/install.sh')
PY
}
select_venv_recreation() {
  # Check-only never repairs or moves the current environment.
  if ((!check_only && !recreate)) && [[ -e "$VENV" || -L "$VENV" ]]; then
    local reason
    if ! reason="$(validate_venv 2>&1)"; then
      recreate=1
      printf 'Existing .venv is incompatible; it will be backed up and rebuilt.\n%s\n' "$reason"
    fi
  fi
}
prepare_venv() {
  info 'Independent Python environment'
  if ((recreate)) && [[ -e "$VENV" || -L "$VENV" ]]; then
    run mv -- "$VENV" "$VENV.backup-$(date +%Y%m%d-%H%M%S)-$$"
  fi
  if ((recreate)) || [[ ! -e "$VENV" ]]; then
    run env -u PYTHONPATH -u PYTHONHOME "$PYTHON" -m venv "$VENV"
  else
    validate_venv
  fi
  local extras=go2
  if [[ "$plugin" == path_tracker || "$plugin" == all ]]; then extras+=,path_tracker; fi
  if [[ "$plugin" == agent || "$plugin" == all ]]; then extras+=,agent; fi
  local -a command
  if command -v uv >/dev/null 2>&1; then
    command=(uv pip install --python "$VENV/bin/python")
    ((offline == 0)) || command+=(--offline)
  else
    command=("$VENV/bin/python" -m pip install)
    if ((offline)); then
      [[ -d "${DIMOS_WHEELHOUSE:-}" ]] || die 'pip offline mode requires DIMOS_WHEELHOUSE (including build requirements)'
      command+=(--no-index --find-links "$DIMOS_WHEELHOUSE")
    fi
  fi
  run env -u PYTHONPATH -u PYTHONHOME "${command[@]}" \
    --constraint "$ROOT/dimos/requirements.lock" -e "$ROOT/dimos[$extras]"
}
sdk_ready() {
  local p="$WS/third_party/insta360_ros_driver"
  [[ -f "$p/lib/libCameraSDK.so" && -f "$p/include/camera/camera.h" &&
     -f "$p/include/stream/stream_delegate.h" && -f "$p/lib/jsons/camera_conf_Insta360_X3.json" ]]
}
extract_sdk() {
  # Validate archive paths before extraction; reject links and special files.
  "$PYTHON" - "$1" "$2" <<'PY'
import pathlib, sys, tarfile, zipfile
archive, destination = sys.argv[1:]
root = pathlib.Path(destination).resolve()
def safe(name):
    path = root / name
    if not path.resolve().is_relative_to(root):
        raise SystemExit('Unsafe SDK archive path')
if zipfile.is_zipfile(archive):
    with zipfile.ZipFile(archive) as z:
        for item in z.infolist():
            safe(item.filename)
            if (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise SystemExit('SDK archive contains symbolic links; use --sdk-dir')
        z.extractall(root)
else:
    with tarfile.open(archive) as t:
        for item in t.getmembers():
            safe(item.name)
            if not (item.isfile() or item.isdir()):
                raise SystemExit('SDK archive contains links/special files; use --sdk-dir')
        t.extractall(root, filter='data')
PY
}
prepare_sdk() {
  [[ "$camera" == x3 || "$camera" == all ]] || return 0
  info 'Insta360 SDK'
  if sdk_ready && [[ -z "$sdk_dir$sdk_archive$sdk_url" ]]; then
    printf 'Using existing CameraSDK headers/library/config.\n'
    return
  fi
  if ((dry_run)); then
    printf '  Prepare CameraSDK from --sdk-dir/--sdk-archive/--sdk-url or existing files; validate architecture.\n'
    return
  fi
  [[ -n "$sdk_dir$sdk_archive$sdk_url" ]] || die 'X3 needs CameraSDK; supply --sdk-dir, --sdk-archive or an authorized --sdk-url'
  local count=0 source
  for source in "$sdk_dir" "$sdk_archive" "$sdk_url"; do [[ -z "$source" ]] || count=$((count + 1)); done
  ((count == 1)) || die 'specify only one SDK source'
  if [[ -n "$sdk_url" ]]; then
    mkdir -p "$WS/downloads"
    # A URL-specific cache avoids silently reusing a different SDK with the same filename.
    local key destination
    key="$(printf '%s' "$sdk_url" | sha256sum | cut -d' ' -f1)"
    destination="$WS/downloads/CameraSDK-$key.archive"
    if [[ ! -f "$destination" ]]; then
      curl -fL --retry 3 "$sdk_url" -o "$destination.part"
      mv -- "$destination.part" "$destination"
    fi
    sdk_archive="$destination"
  fi
  if [[ -n "$sdk_archive" ]]; then
    [[ -f "$sdk_archive" ]] || die 'SDK archive does not exist'
    if [[ -n "$sdk_sha256" ]]; then
      printf '%s  %s\n' "$sdk_sha256" "$sdk_archive" | sha256sum -c -
    fi
    mkdir -p "$temp/sdk"
    extract_sdk "$sdk_archive" "$temp/sdk"
    sdk_dir="$temp/sdk"
    # Official bundles may contain architecture-specific nested archives.
    if ! find "$sdk_dir" -type f -path '*/include/camera/camera.h' -print -quit | read -r _; then
      local -a nested=()
      mapfile -t nested < <(find "$sdk_dir" -type f \( -iname '*aarch64*.tar*' -o -iname '*aarch64*.tgz' \) | sort)
      local selected=''
      for source in "${nested[@]}"; do [[ "${source,,}" != *jetson* ]] || selected="$source"; done
      if [[ -z "$selected" ]]; then
        ((${#nested[@]} == 1)) || die 'ambiguous nested SDK archives; extract the matching ARM64 SDK and use --sdk-dir'
        selected="${nested[0]}"
      fi
      mkdir -p "$temp/selected-sdk"
      extract_sdk "$selected" "$temp/selected-sdk"
      sdk_dir="$temp/selected-sdk"
    fi
  elif [[ -n "$sdk_sha256" ]]; then
    die '--sdk-sha256 applies to an archive or URL, not --sdk-dir'
  fi
  local -a headers=()
  mapfile -t headers < <(find "$sdk_dir" -type f -path '*/include/camera/camera.h' | sort)
  ((${#headers[@]} == 1)) || die 'SDK directory must contain exactly one SDK; select the correct architecture subdirectory'
  local sdk_root p="$WS/third_party/insta360_ros_driver"
  sdk_root="${headers[0]%/include/camera/camera.h}"
  [[ -f "$sdk_root/lib/libCameraSDK.so" && -f "$sdk_root/include/stream/stream_delegate.h" &&
     -f "$sdk_root/lib/jsons/camera_conf_Insta360_X3.json" ]] || die 'incomplete CameraSDK headers/library/X3 JSON'
  LC_ALL=C readelf -h "$sdk_root/lib/libCameraSDK.so" | grep -q 'Machine:.*AArch64' || die 'CameraSDK must be AArch64'
  mkdir -p "$p/include/camera" "$p/include/stream" "$p/lib/jsons"
  cp -a "$sdk_root/include/camera/." "$p/include/camera/"
  cp -a "$sdk_root/include/stream/." "$p/include/stream/"
  install -m0755 "$sdk_root/lib/libCameraSDK.so" "$p/lib/libCameraSDK.so"
  cp -a "$sdk_root/lib/jsons/." "$p/lib/jsons/"
  sdk_ready || die 'CameraSDK installation is incomplete'
}
build_cameras() {
  [[ "$camera" != none ]] || return 0
  ((skip_build == 0)) || return 0
  info 'Build camera workspace'
  local -a targets=(insta360_ros_driver) cmake_args=()
  if [[ "$camera" == d455 || "$camera" == all ]]; then targets+=(realsense2_camera compressed_RGBD_pkg); fi
  if [[ "$camera" == d455 ]]; then cmake_args+=(-DBUILD_INSTA360_CAMERA=OFF); else cmake_args+=(-DBUILD_INSTA360_CAMERA=ON); fi
  if ((dry_run)); then
    printf '  source camera_ws/sh/setup_local.bash; colcon build --packages-up-to %s --cmake-args %s\n' "${targets[*]}" "${cmake_args[*]}"
    return
  fi
  (
    unset VIRTUAL_ENV PYTHONHOME
    export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    export ROS_DISTRO=jazzy
    cd "$WS"
    set +u
    source sh/setup_local.bash
    set -u
    colcon build --base-paths third_party src --packages-up-to "${targets[@]}" --parallel-workers 2 \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF -DPython3_EXECUTABLE=/usr/bin/python3 "${cmake_args[@]}"
  )
}
check_lifecycle_log() {
  cat "$temp/lifecycle.log"
  (($1 == 0)) || die "ROS lifecycle failed (exit $1)"
  if grep -Eq 'RTPS.*Error|DDS.*Error' "$temp/lifecycle.log"; then
    die "DDS reported an error despite a zero exit status; inspect $FASTRTPS_DEFAULT_PROFILES_FILE and remote DDS settings"
  fi
}
verify() {
  info 'Environment verification'
  if ((dry_run)); then
    printf '  Check isolated venv, editable path, dependency pins/imports, ROS types/DDS, selected camera libraries and launch files; run empty ROS lifecycle; validate default XNavDP/tracker configs and MPC.\n'
    return
  fi
  validate_venv
  # Resolve the editable package without PYTHONPATH masking an incorrect installation.
  env -u PYTHONPATH -u PYTHONHOME "$VENV/bin/python" - "$ROOT" "$plugin" <<'PY'
from importlib import import_module, metadata
from pathlib import Path
from urllib.parse import unquote, urlparse
import json, sys
root = Path(sys.argv[1]).resolve()
dist = metadata.distribution('dimos-mininum')
direct = json.loads(dist.read_text('direct_url.json') or '{}')
if (Path(unquote(urlparse(direct.get('url', '')).path)).resolve() != root / 'dimos'
        or not direct.get('dir_info', {}).get('editable')):
    raise SystemExit('DimOS editable installation points outside this workspace')
import dimos
if Path(dimos.__file__).resolve() != root / 'dimos/__init__.py':
    raise SystemExit('Another dimos package shadows this workspace')
for module in ('yaml', 'numpy', 'aiortc', 'aioice', 'wasmtime',
               'unitree_webrtc_connect.webrtc_driver', 'dimos.robot.unitree.connection'):
    import_module(module)
if sys.argv[2] in {'path_tracker', 'all'}:
    for name in ('numpy', 'scipy', 'yaml'):
        module = import_module(name)
        if not Path(module.__file__).resolve().is_relative_to(root / '.venv'):
            raise SystemExit(f'{name} must be installed inside workspace .venv')
    from dimos.plugins.path_tracker.__main__ import check_dependencies
    check_dependencies()
    import numpy as np
    from dimos.plugins.path_tracker.mpc import solve_mpc
    commands = solve_mpc(np.array([[0.1, 0, 0], [0.5, 0, 0]]), 15, 0.1, 0.4, 0.4, 0.2)
    if commands.shape != (15, 2) or not np.isfinite(commands).all():
        raise SystemExit('Tracker MPC smoke check failed')
    print('PASS: workspace tracker dependency binding and MPC solve')
if sys.argv[2] in {'agent', 'all'}:
    for name in ('httpx', 'PIL', 'fastapi', 'python_multipart', 'uvicorn', 'cv2', 'websocket'):
        module = __import__(name)
        if not Path(module.__file__).resolve().is_relative_to(root / '.venv'):
            raise SystemExit(f'Agent dependency resolves outside workspace venv: {name}')
    from dimos.plugins._agent_cli import main
    for name in ('vlm', 'llm', 'vln_task', 'web_input'):
        if main(name, ['--check']) != 0:
            raise SystemExit(f'Agent configuration check failed: {name}')
    print('PASS: independent agent dependencies and configs; no model connections')
# Check installed constrained dependencies, including SDK transitive dependencies.
for line in (root / 'dimos/requirements.lock').read_text().splitlines():
    if '==' not in line or line.startswith('#'):
        continue
    name, expected = line.split('==', 1)
    try:
        actual = metadata.version(name)
    except metadata.PackageNotFoundError:
        continue
    if actual != expected:
        raise SystemExit(f'{name}: expected {expected}, found {actual}')
print('PASS: venv, editable package, SDK imports and installed dependency pins')
PY
  if command -v uv >/dev/null 2>&1; then
    UV_CACHE_DIR="$temp/uv-cache" env -u PYTHONPATH -u PYTHONHOME uv pip check --python "$VENV/bin/python"
  else
    env -u PYTHONPATH -u PYTHONHOME "$VENV/bin/python" -m pip check
  fi
  (
    set +u
    export ROS_DISTRO=jazzy ROS_LOG_DIR="$temp/ros-log"
    source "$ROOT/sh/env.sh"
    if [[ "$camera" != none ]]; then source "$WS/install/local_setup.bash"; fi
    set -u
    "$VENV/bin/python" - <<'PY'
from pathlib import Path
import os, xml.etree.ElementTree as ET
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2, CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from rosidl_generator_py import import_type_support
for package in ('geometry_msgs', 'sensor_msgs', 'std_msgs', 'std_srvs', 'tf2_msgs'):
    import_type_support(package)
ET.parse(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'])
if not Path(rclpy.__file__).resolve().is_relative_to(Path(os.environ['INSTA360_ROOT']).resolve()):
    raise SystemExit('rclpy resolves outside camera_ws')
print('PASS: ROS Python types, native type support and DDS XML')
PY
    "$VENV/bin/python" -m dimos --check
    "$VENV/bin/python" -m dimos.plugins.xnavdp --check
    if [[ "$plugin" == path_tracker || "$plugin" == all ]]; then
      "$VENV/bin/python" -m dimos.plugins.path_tracker --check
    fi
    # No camera/robot modules are enabled in empty.yaml.
    local lifecycle_status=0
    timeout 20 "$VENV/bin/python" -m dimos --config "$ROOT/config/empty.yaml" --duration 1 \
      >"$temp/lifecycle.log" 2>&1 || lifecycle_status=$?
    check_lifecycle_log "$lifecycle_status"
    if [[ "$camera" != none ]]; then verify_cameras; fi
  )
  printf '\nPASS: software environment (%s). Live device data was not tested.\n' "$camera"
}
verify_cameras() {
  local -a packages=(insta360_ros_driver) binaries=() launches=()
  local path prefix output
  /usr/bin/python3 -c 'import cv2, numpy, launch, launch_ros, insta360_ros_driver.camera_state'
  if [[ "$camera" == d455 || "$camera" == all ]]; then
    packages+=(realsense2_camera compressed_RGBD_pkg)
    binaries+=("$WS/install/realsense2_camera/lib/librealsense2_camera.so"
      "$WS/install/compressed_RGBD_pkg/lib/compressed_RGBD_pkg/compressed_rgbd_node")
    launches+=("$WS/launch/start_d455.launch.py" "$WS/launch/compressed_d455.launch.py")
  fi
  if [[ "$camera" == x3 || "$camera" == all ]]; then
    sdk_ready || die 'CameraSDK missing'
    for path in insta360_ros_driver decoder equirectangular_cpp surround_views; do
      binaries+=("$WS/install/insta360_ros_driver/lib/insta360_ros_driver/$path")
    done
    launches+=("$WS/install/insta360_ros_driver/share/insta360_ros_driver/launch/bringup.launch.xml")
    path="$WS/install/insta360_ros_driver/share/insta360_ros_driver/lib"
    cmp -s "$WS/third_party/insta360_ros_driver/lib/libCameraSDK.so" "$path/libCameraSDK.so" || die 'installed CameraSDK differs from source; rebuild'
    cmp -s "$WS/third_party/insta360_ros_driver/lib/jsons/camera_conf_Insta360_X3.json" "$path/jsons/camera_conf_Insta360_X3.json" || die 'installed X3 JSON differs from source; rebuild'
  fi
  for path in "${packages[@]}"; do
    prefix="$(ros2 pkg prefix "$path")"
    [[ "$prefix" == "$WS/install/"* ]] || die "ROS package resolves outside camera_ws/install: $path"
  done
  for path in "${binaries[@]}"; do
    [[ -f "$path" ]] || die "missing camera binary: $path"
    output="$(ldd "$path" 2>&1)" || die "ldd failed: $path"
    [[ "$output" != *'not found'* ]] || die "unresolved shared library: $path: $output"
    if printf '%s\n' "$output" | /usr/bin/python3 -c '
import os, re, sys
from pathlib import Path
ws = Path(os.environ["INSTA360_ROOT"]).resolve()
for lib in re.findall(r"=> (/\S+)", sys.stdin.read()):
    p = Path(lib).resolve()
    if (lib.startswith("/opt/ros/") or lib.startswith(str(Path.home()) + "/")) and not p.is_relative_to(ws):
        raise SystemExit(1)
'; then :; else die "external workspace/ROS library used by $path"; fi
  done
  for path in "${launches[@]}"; do timeout 20 ros2 launch "$path" --show-args >/dev/null; done
  printf 'PASS: selected camera packages, shared libraries and launch descriptions\n'
}
readiness() {
  info 'Device readiness (not a live communication test)'
  ROOT="$ROOT" "$VENV/bin/python" - <<'PY'
from dimos.config import load_config
from dimos.robot.unitree.go2.body_bridge import read_aes_key
from dimos.robot.unitree.go2.wired_link import WiredLink
import os
cfg = load_config(os.path.join(os.environ['ROOT'], 'config/base.yaml'))
try:
    read_aes_key(cfg['body']['aes_key_env'])
    print('READY: Go2 credential format')
except RuntimeError:
    print('PENDING: Go2 credential missing/invalid; configure ~/.config/dimos/unitree.env')
os.environ['DIMOS_GO2_WIRED_INTERFACE'] = cfg['body']['interface']
try:
    WiredLink.check(cfg['body']['ip'])
    print('READY: Go2 Ethernet link/address/route')
except (RuntimeError, OSError):
    print('PENDING: Go2 Ethernet link/address/route; check configured interface and robot IP')
PY
  if [[ "$camera" == x3 || "$camera" == all ]]; then
    if [[ -r /dev/insta && -w /dev/insta ]]; then printf 'READY: X3 USB access\n';
    else printf 'PENDING: X3 USB device/permissions; configure udev and reconnect in Android USB mode\n'; fi
  fi
  if [[ "$camera" == d455 || "$camera" == all ]]; then
    printf 'PENDING: D455 USB permissions and RGB/depth/CameraInfo data need a live camera test\n'
  fi
}

preflight
system_dependencies
if ((!dry_run)); then
  temp="$(mktemp -d /tmp/dimos-install.XXXXXX)"
  export PYTHONDONTWRITEBYTECODE=1
fi
if ((!check_only)); then
  select_venv_recreation
  prepare_sdk
  build_cameras
  prepare_venv
fi
verify
if ((dry_run)); then
  printf '\nDry-run complete; no installation commands executed.\n'
else
  (
    set +u
    source "$ROOT/sh/env.sh"
    set -u
    readiness
  )
fi

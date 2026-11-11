#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export WORKSPACE_DIR
source "${WORKSPACE_DIR}/config/start.conf"

camera_args=()
prepare_only=0
skip_check=0
install_system_deps="$AUTO_INSTALL_SYSTEM_DEPS"
setup_udev="$AUTO_SETUP_UDEV"
explicit_udev=0
sdk_url="$CAMERA_SDK_URL"
sdk_archive="$CAMERA_SDK_ARCHIVE"
sdk_dir="$CAMERA_SDK_DIR"
sdk_sha256="$CAMERA_SDK_SHA256"

usage() {
  cat <<'EOF'
Usage:
  bash start.sh [options] [camera options]

Bootstrap options:
  --sdk-url URL          Download CameraSDK from a URL
  --sdk-archive PATH     Use an official CameraSDK ZIP/TAR archive
  --sdk-dir PATH         Use an already extracted CameraSDK directory
  --sdk-sha256 HEX       Verify the downloaded SDK archive
  --install-system-deps  Install required Ubuntu packages with apt/sudo
  --setup-udev           Configure the Insta360 X3 USB/udev rule
  --prepare-only         Configure and validate, but do not launch cameras
  --skip-check           Skip the workspace independence self-check
  -h, --help             Show this help

Camera options (passed to start_cameras.sh):
  --camera x3
  --camera d455
  --camera x3 d455
  --status
  --dry-run

Examples:
  bash start.sh
  bash start.sh --camera x3
  bash start.sh --sdk-archive ~/CameraSDK.zip --prepare-only
  CAMERA_SDK_URL=https://example/sdk.zip bash start.sh --camera d455
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

cleanup_paths=()
cleanup() {
  local path
  for path in "${cleanup_paths[@]}"; do
    [[ -n "$path" ]] && rm -rf -- "$path"
  done
}
trap cleanup EXIT

while (($#)); do
  case "$1" in
    --sdk-url)
      sdk_url="${2:?missing URL}"
      shift 2
      ;;
    --sdk-archive)
      sdk_archive="${2:?missing archive path}"
      shift 2
      ;;
    --sdk-dir)
      sdk_dir="${2:?missing SDK directory}"
      shift 2
      ;;
    --sdk-sha256)
      sdk_sha256="${2:?missing SHA256 value}"
      shift 2
      ;;
    --install-system-deps)
      install_system_deps=1
      shift
      ;;
    --setup-udev)
      setup_udev=1
      explicit_udev=1
      shift
      ;;
    --prepare-only)
      prepare_only=1
      shift
      ;;
    --skip-check)
      skip_check=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --camera)
      camera_args+=("--camera")
      shift
      while (($#)) && [[ "$1" != --* ]]; do
        camera_args+=("$1")
        shift
      done
      ;;
    --status|--status-demo|--color-demo|--dry-run|--d455|--x3)
      camera_args+=("$1")
      shift
      ;;
    *)
      die "unknown option: $1. Run bash start.sh --help."
      ;;
  esac
done

needs_camera=1
needs_sdk=1
needs_x3=0
for arg in "${camera_args[@]}"; do
  case "${arg,,}" in
    --status|--status-demo|--color-demo|--dry-run) needs_camera=0;;
    x3|--x3) needs_x3=1;;
  esac
done
if ((${#camera_args[@]} == 0)); then
  needs_x3=1
fi
if ((prepare_only)); then
  needs_camera=0
  needs_sdk=1
fi
if ((needs_camera)); then
  needs_sdk=1
elif ((!prepare_only)); then
  needs_sdk=0
fi

install_system_packages() {
  command -v apt-get >/dev/null || die "apt-get is required to install system dependencies"
  local sudo_cmd=()
  if ((EUID != 0)); then
    command -v sudo >/dev/null || die "sudo is required to install system dependencies"
    sudo_cmd=(sudo)
  fi
  "${sudo_cmd[@]}" apt-get update
  "${sudo_cmd[@]}" apt-get install -y \
    ca-certificates curl unzip python3 python3-numpy python3-opencv \
    ffmpeg libusb-1.0-0 libudev1 usbutils udev
}

if ((install_system_deps)); then
  install_system_packages
fi

for command_name in bash tar python3; do
  command -v "$command_name" >/dev/null ||
    die "missing required command: $command_name"
done

mkdir -p "$WORKSPACE_DIR/log"

source "${WORKSPACE_DIR}/sh/setup_local.bash"
set +u
source "${WORKSPACE_DIR}/install/local_setup.bash"
set -u

if ! python3 -c 'import cv2, numpy' >/dev/null 2>&1; then
  die "Python OpenCV/NumPy is missing. Run: bash start.sh --install-system-deps --prepare-only"
fi

sdk_ready() {
  local package_dir="${WORKSPACE_DIR}/third_party/insta360_ros_driver"
  local installed_dir="${WORKSPACE_DIR}/install/insta360_ros_driver/share/insta360_ros_driver"
  [[ -f "${package_dir}/lib/libCameraSDK.so" &&
     -f "${package_dir}/include/camera/camera.h" &&
     -f "${package_dir}/include/stream/stream_delegate.h" &&
     -f "${package_dir}/lib/jsons/camera_conf_Insta360_X3.json" &&
     -f "${installed_dir}/lib/libCameraSDK.so" &&
     -f "${installed_dir}/lib/jsons/camera_conf_Insta360_X3.json" ]]
}

sync_sdk_runtime() {
  local package_dir="${WORKSPACE_DIR}/third_party/insta360_ros_driver"
  local installed_dir="${WORKSPACE_DIR}/install/insta360_ros_driver/share/insta360_ros_driver"
  [[ -d "$installed_dir" ]] || die "installed camera driver is missing: $installed_dir"
  install -Dm0755 "${package_dir}/lib/libCameraSDK.so" "${installed_dir}/lib/libCameraSDK.so"
  mkdir -p "${installed_dir}/lib/jsons"
  install -m0644 "${package_dir}"/lib/jsons/*.json "${installed_dir}/lib/jsons/"
}

extract_archive() {
  local archive="$1" destination="$2" lower
  lower="${archive,,}"
  mkdir -p "$destination"
  case "$lower" in
    *.zip)
      command -v unzip >/dev/null || die "unzip is required for $archive"
      unzip -q "$archive" -d "$destination"
      ;;
    *.tar.gz|*.tgz|*.tar*.gz)
      tar -xzf "$archive" -C "$destination"
      ;;
    *.tar.xz|*.txz)
      tar -xJf "$archive" -C "$destination"
      ;;
    *.tar)
      tar -xf "$archive" -C "$destination"
      ;;
    *)
      local kind
      kind="$(file -b "$archive" 2>/dev/null || true)"
      case "$kind" in
        *Zip*) command -v unzip >/dev/null || die "unzip is required for $archive"; unzip -q "$archive" -d "$destination";;
        *gzip*) tar -xzf "$archive" -C "$destination";;
        *) die "unsupported SDK archive format: $archive";;
      esac
      ;;
  esac
}

find_sdk_root() {
  local search_root="$1" header sdk_root
  header="$(find "$search_root" -type f -path '*/include/camera/camera.h' -print -quit)"
  [[ -n "$header" ]] || return 1
  sdk_root="$(dirname -- "$(dirname -- "$(dirname -- "$header")")")"
  [[ -f "${sdk_root}/lib/libCameraSDK.so" &&
     -f "${sdk_root}/include/stream/stream_delegate.h" ]] || return 1
  printf '%s\n' "$sdk_root"
}

choose_nested_sdk_archive() {
  local search_root="$1" archive host
  host="$(uname -m)"
  local -a archives=()
  mapfile -t archives < <(find "$search_root" -type f \
    \( -name '*.tar.gz' -o -name '*.tgz' -o -name '*.tar*.gz' \) -print | sort)
  ((${#archives[@]})) || return 1
  if [[ "$host" == aarch64 ]]; then
    for archive in "${archives[@]}"; do
      [[ "${archive,,}" == *jetson*aarch64* ]] && { printf '%s\n' "$archive"; return 0; }
    done
    for archive in "${archives[@]}"; do
      [[ "${archive,,}" == *aarch64* ]] && { printf '%s\n' "$archive"; return 0; }
    done
  else
    for archive in "${archives[@]}"; do
      [[ "${archive,,}" == *x86_64* && "${archive,,}" != *aarch64* ]] &&
        { printf '%s\n' "$archive"; return 0; }
    done
  fi
  printf '%s\n' "${archives[0]}"
}

install_sdk_archive() {
  local archive="$1" outer nested sdk_root nested_archive
  mkdir -p "$SDK_CACHE_DIR"
  outer="$(mktemp -d "${SDK_CACHE_DIR}/sdk-extract.XXXXXX")"
  cleanup_paths+=("$outer")
  extract_archive "$archive" "$outer"
  if sdk_root="$(find_sdk_root "$outer")"; then
    bash "${WORKSPACE_DIR}/sh/setup/install_camera_sdk.sh" "$sdk_root"
    return
  fi
  nested_archive="$(choose_nested_sdk_archive "$outer")" ||
    die "could not find CameraSDK files or a supported nested archive in $archive"
  nested="${outer}/selected-sdk"
  extract_archive "$nested_archive" "$nested"
  sdk_root="$(find_sdk_root "$nested")" ||
    die "the selected SDK archive does not contain the required headers and library: $nested_archive"
  bash "${WORKSPACE_DIR}/sh/setup/install_camera_sdk.sh" "$sdk_root"
}

download_sdk() {
  local url="$1" filename destination
  mkdir -p "$SDK_CACHE_DIR"
  filename="${url%%\?*}"
  filename="${filename##*/}"
  [[ -n "$filename" && "$filename" != "$url" ]] || filename='CameraSDK.zip'
  destination="${SDK_CACHE_DIR}/${filename}"
  if [[ ! -f "$destination" ]]; then
    if command -v curl >/dev/null; then
      curl -fL --retry 3 --continue-at - "$url" -o "$destination"
    elif command -v wget >/dev/null; then
      wget -c "$url" -O "$destination"
    else
      die "curl or wget is required to download CameraSDK"
    fi
  fi
  if [[ -n "$sdk_sha256" ]]; then
    printf '%s  %s\n' "$sdk_sha256" "$destination" | sha256sum -c -
  fi
  sdk_archive="$destination"
}

if ((needs_sdk)) && ! sdk_ready && [[ -z "$sdk_dir" && -z "$sdk_archive" && -z "$sdk_url" ]]; then
  sdk_candidate="$(find "$WORKSPACE_DIR" -maxdepth 2 -type f \
    \( -iname '*CameraSDK*.zip' -o -iname '*CameraSDK*.tar*' \) \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  if [[ -n "$sdk_candidate" ]]; then
    sdk_archive="$sdk_candidate"
    printf 'Found CameraSDK archive: %s\n' "$sdk_archive"
  else
    sdk_candidate="$(find "$WORKSPACE_DIR" -maxdepth 2 -type d -name 'CameraSDK-*' \
      -print -quit 2>/dev/null)"
    if [[ -n "$sdk_candidate" ]]; then
      sdk_dir="$sdk_candidate"
      printf 'Found CameraSDK directory: %s\n' "$sdk_dir"
    fi
  fi
fi

if ((needs_sdk)) && ! sdk_ready; then
  if [[ -n "$sdk_dir" ]]; then
    [[ -d "$sdk_dir" ]] || die "CameraSDK directory not found: $sdk_dir"
    bash "${WORKSPACE_DIR}/sh/setup/install_camera_sdk.sh" "$sdk_dir"
  elif [[ -n "$sdk_archive" ]]; then
    [[ -f "$sdk_archive" ]] || die "CameraSDK archive not found: $sdk_archive"
    install_sdk_archive "$sdk_archive"
  elif [[ -n "$sdk_url" ]]; then
    download_sdk "$sdk_url"
    install_sdk_archive "$sdk_archive"
  elif [[ -t 0 ]]; then
    printf '\nCameraSDK is required for first-time setup.\n'
    printf 'Download it from https://www.insta360.com/sdk/home, then enter a ZIP/TAR archive or extracted directory.\n'
    read -r -p 'CameraSDK path (leave blank to cancel): ' sdk_input
    [[ -n "$sdk_input" ]] || die "CameraSDK setup cancelled"
    if [[ -d "$sdk_input" ]]; then
      bash "${WORKSPACE_DIR}/sh/setup/install_camera_sdk.sh" "$sdk_input"
    elif [[ -f "$sdk_input" ]]; then
      install_sdk_archive "$sdk_input"
    else
      die "CameraSDK path not found: $sdk_input"
    fi
  else
    die "CameraSDK is missing. Set CAMERA_SDK_URL in config/start.conf, pass --sdk-archive/--sdk-dir, or run start.sh interactively"
  fi
fi

if ((needs_sdk)); then
  sdk_ready || die "CameraSDK installation is incomplete"
  package_lib="${WORKSPACE_DIR}/third_party/insta360_ros_driver/lib/libCameraSDK.so"
  installed_lib="${WORKSPACE_DIR}/install/insta360_ros_driver/share/insta360_ros_driver/lib/libCameraSDK.so"
  package_json="${WORKSPACE_DIR}/third_party/insta360_ros_driver/lib/jsons/camera_conf_Insta360_X3.json"
  installed_json="${WORKSPACE_DIR}/install/insta360_ros_driver/share/insta360_ros_driver/lib/jsons/camera_conf_Insta360_X3.json"
  if ! cmp -s "$package_lib" "$installed_lib" || ! cmp -s "$package_json" "$installed_json"; then
    sync_sdk_runtime
  fi
fi

if ((needs_x3 && (needs_camera || setup_udev))) && [[ ! -e /dev/insta ]]; then
  should_setup_udev="$setup_udev"
  if [[ "$should_setup_udev" != 1 && -t 0 ]]; then
    read -r -p 'Insta360 USB access is not configured. Run the udev setup now? [y/N] ' answer
    [[ "${answer,,}" == y || "${answer,,}" == yes ]] && should_setup_udev=1
  fi
  if [[ "$should_setup_udev" == 1 ]]; then
    if ! bash "${WORKSPACE_DIR}/third_party/insta360_ros_driver/setup.sh"; then
      if ((explicit_udev)); then
        die "Insta360 udev setup was requested but did not complete"
      fi
      printf 'WARNING: Insta360 udev setup did not complete. X3 access may fail.\n' >&2
    fi
  elif ((explicit_udev)); then
    die "Insta360 udev setup was requested but did not complete"
  else
    printf 'WARNING: /dev/insta is missing. X3 access may fail; rerun with --setup-udev.\n' >&2
  fi
fi

if ((needs_sdk && !skip_check && RUN_STANDALONE_CHECK)); then
  bash "${WORKSPACE_DIR}/sh/maintenance/check_standalone.sh"
fi

if ((prepare_only)); then
  printf 'Camera workspace setup completed: %s\n' "$WORKSPACE_DIR"
  exit 0
fi

exec bash "${WORKSPACE_DIR}/start_cameras.sh" "${camera_args[@]}"

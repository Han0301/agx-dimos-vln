# Shared DDS settings for camera publishers and DimOS.
# DIMOS_XNAVDP_ROOT can override the parent workspace when needed.
_camera_config_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
_dimos_root="${DIMOS_XNAVDP_ROOT:-$(cd -- "${_camera_config_dir}/../.." && pwd -P)}"
source "${_dimos_root}/sh/xnavdp_ros2/dds_env.sh"
unset _camera_config_dir _dimos_root

#!/usr/bin/env bash
# Install the DimOS environment used by the Go2 VLN workspace.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$WORKSPACE_ROOT/.venv"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/config.env"

CHECK_ONLY=0
SKIP_SYSTEM=0
RECREATE=0
SKIP_VERIFY=0
DRY_RUN=0
OFFLINE=0

usage() {
  cat <<'EOF'
DimOS 一键虚拟环境安装器

用法:
  bash sh/start_venv.sh [选项]

选项:
  --check-only       只检查前置条件和现有环境，不修改文件
  --recreate         将现有 .venv 备份后重新创建
  --skip-system      跳过 Ubuntu 系统依赖安装
  --skip-verify      安装完成后跳过校验
  --offline          使用 uv 离线模式安装
  --dry-run          只显示将执行的安装步骤
  -h, --help         显示帮助

默认安装:
  unitree + navdp extras，Python 3.12，项目本地 .venv
EOF
}

while (($#)); do
  case "$1" in
    --check-only) CHECK_ONLY=1 ;;
    --skip-system) SKIP_SYSTEM=1 ;;
    --recreate) RECREATE=1 ;;
    --skip-verify) SKIP_VERIFY=1 ;;
    --offline) OFFLINE=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知选项: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

info() { printf '\033[36m▸\033[0m %s\n' "$*"; }
ok() { printf '\033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '\033[33m⚠\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31m✗\033[0m %s\n' "$*" >&2; exit 1; }

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

run() {
  if ((DRY_RUN)); then
    print_command "$@"
  else
    "$@"
  fi
}

run_uv_sync() {
  local -a command=(
    uv sync
    --project "$WORKSPACE_ROOT"
    --python "$VENV_DIR/bin/python"
    --locked
  )
  local extra
  for extra in "${DIMOS_EXTRAS[@]}"; do
    command+=(--extra "$extra")
  done
  if ((OFFLINE)); then
    command+=(--offline)
  fi
  if ((DRY_RUN)); then
    print_command "${command[@]}"
  else
    (cd "$WORKSPACE_ROOT" && "${command[@]}")
  fi
}

check_prerequisites() {
  info "检查工作空间"
  [[ -f "$WORKSPACE_ROOT/pyproject.toml" ]] || die "找不到 $WORKSPACE_ROOT/pyproject.toml"
  [[ -f "$WORKSPACE_ROOT/uv.lock" ]] || die "找不到 $WORKSPACE_ROOT/uv.lock"
  [[ -f "$WORKSPACE_ROOT/dimos/robot/cli/dimos.py" ]] || die "DimOS 源码不完整"
  ok "工作空间: $WORKSPACE_ROOT"

  local navdp_bridge="$WORKSPACE_ROOT/../NavDP/ros2_ws/src/navdp_bridge"
  [[ -f "$navdp_bridge/setup.py" ]] ||
    die "找不到本地 navdp_bridge: $navdp_bridge"
  ok "NavDP bridge: $navdp_bridge"

  export PATH="$HOME/.local/bin:$PATH"
  command -v uv >/dev/null 2>&1 || die "未找到 uv。请先安装 uv，再运行本脚本。"
  ok "uv: $(uv --version)"

  command -v git >/dev/null 2>&1 || warn "未找到 git；后续将尝试通过系统包安装"
  command -v g++ >/dev/null 2>&1 || warn "未找到 g++；后续将尝试通过 build-essential 安装"

  if [[ -f "$ROS2_BRIDGE_ROOT/env.sh" && -f "$ROS2_JAZZY/install/local_setup.bash" ]]; then
    ok "ROS 2 Jazzy: $ROS2_JAZZY"
  else
    warn "ROS 2 bridge 不完整；D455i 启动前需配置 ROS2_BRIDGE_ROOT 和 ROS2_JAZZY"
  fi

  local available_mb
  available_mb="$(df -Pm "$WORKSPACE_ROOT" | awk 'NR == 2 {print $4}')"
  if [[ "$available_mb" =~ ^[0-9]+$ ]] && ((available_mb < DIMOS_MIN_FREE_MB)); then
    warn "磁盘剩余 ${available_mb} MiB，低于建议的 ${DIMOS_MIN_FREE_MB} MiB"
  else
    ok "磁盘剩余: ${available_mb:-unknown} MiB"
  fi
}

install_system_packages() {
  local -a missing=()
  local package
  if command -v dpkg-query >/dev/null 2>&1; then
    for package in "${SYSTEM_PACKAGES[@]}"; do
      if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null |
        grep -q 'install ok installed'; then
        missing+=("$package")
      fi
    done
  else
    warn "当前系统没有 dpkg-query，跳过系统包检查"
    return 0
  fi

  if ((${#missing[@]} == 0)); then
    ok "Ubuntu 系统依赖已安装"
    return 0
  fi

  warn "缺少系统包: ${missing[*]}"
  if ((CHECK_ONLY)); then
    return 1
  fi
  if ((SKIP_SYSTEM)); then
    warn "已指定 --skip-system，继续安装 Python 依赖"
    return 0
  fi
  command -v apt-get >/dev/null 2>&1 || die "缺少 apt-get，请手动安装: ${missing[*]}"

  local -a apt_command=(apt-get update)
  if ((EUID != 0)); then
    command -v sudo >/dev/null 2>&1 || die "需要 sudo 安装系统包"
    apt_command=(sudo "${apt_command[@]}")
  fi
  run "${apt_command[@]}"

  apt_command=(apt-get install -y "${missing[@]}")
  if ((EUID != 0)); then
    apt_command=(sudo "${apt_command[@]}")
  fi
  run "${apt_command[@]}"
}

check_build_tools() {
  command -v git >/dev/null 2>&1 || die "git 仍不可用"
  command -v g++ >/dev/null 2>&1 || die "g++ 仍不可用"
  ok "编译工具可用"
}

prepare_license() {
  local target="$WORKSPACE_ROOT/other/legal/LICENSE"
  if [[ -f "$target" ]]; then
    ok "许可证文件已存在"
    return 0
  fi
  if ((CHECK_ONLY)); then
    warn "许可证文件尚未创建: $target"
    return 1
  fi
  info "补齐 pyproject.toml 声明的许可证文件"
  run install -Dm0644 "$SCRIPT_DIR/LICENSE" "$target"
}

backup_existing_venv() {
  local backup="$VENV_DIR.backup-$(date +%Y%m%d-%H%M%S)"
  info "备份现有 .venv 到: $backup"
  run mv "$VENV_DIR" "$backup"
}

prepare_venv() {
  if ((RECREATE)) && [[ -e "$VENV_DIR" ]]; then
    backup_existing_venv
    if ((DRY_RUN)); then
      info "重新创建 Python $DIMOS_PYTHON_VERSION 虚拟环境"
      run uv venv --python "$DIMOS_PYTHON_VERSION" "$VENV_DIR"
      return 0
    fi
  fi

  if [[ -e "$VENV_DIR" ]]; then
    if [[ ! -x "$VENV_DIR/bin/python" ]]; then
      die "现有 .venv 不完整；使用 bash sh/start_venv.sh --recreate 重建"
    fi
    local installed_version
    installed_version="$("$VENV_DIR/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    if [[ "$installed_version" != "$DIMOS_PYTHON_VERSION" ]]; then
      die "现有 .venv 使用 Python $installed_version，目标是 $DIMOS_PYTHON_VERSION；请加 --recreate"
    fi
    ok "使用现有 .venv: $VENV_DIR"
  else
    info "创建 Python $DIMOS_PYTHON_VERSION 虚拟环境"
    run uv venv --python "$DIMOS_PYTHON_VERSION" "$VENV_DIR"
  fi
}

main() {
  printf '\nDimOS 环境安装器\n'
  printf '================\n'
  check_prerequisites
  if ! install_system_packages; then
    ((CHECK_ONLY)) || die "系统依赖安装失败"
  fi
  if ((CHECK_ONLY == 0 && SKIP_SYSTEM == 0 && DRY_RUN == 0)); then
    check_build_tools
  fi
  if ! prepare_license; then
    ((CHECK_ONLY)) || die "无法创建许可证文件"
  fi

  if ((CHECK_ONLY)); then
    if [[ -x "$VENV_DIR/bin/python" ]]; then
      info "现有 .venv 检查"
      "$VENV_DIR/bin/python" --version
    else
      die ".venv 尚未创建"
    fi
    if ((SKIP_VERIFY == 0)) && [[ -x "$VENV_DIR/bin/dimos" ]]; then
      bash "$SCRIPT_DIR/verify.sh" --skip-remote
    fi
    ok "检查完成"
    return 0
  fi

  prepare_venv
  info "安装 unitree + navdp 依赖（使用 uv.lock）"
  run_uv_sync
  if ((DRY_RUN)); then
    ok "Dry-run 完成，未修改环境"
    return 0
  fi
  ok "Python 依赖安装完成"

  info "准备 CLIP 图像模型"
  VIRTUAL_ENV="$VENV_DIR" DIMOS_CLIP_OFFLINE="$OFFLINE" "$VENV_DIR/bin/python" "$SCRIPT_DIR/install_clip.py"

  if ((SKIP_VERIFY == 0 && DRY_RUN == 0)); then
    bash "$SCRIPT_DIR/verify.sh" --skip-remote
  fi

  cat <<EOF

安装完成。

激活环境:
  cd "$WORKSPACE_ROOT"
  source ./sh/activate_vln.sh

启动 Go2 VLN:
  dimos --robot-ip 192.168.123.161 --navigation-policy xnavdp run unitree-go2-vln-local
EOF
}

main "$@"

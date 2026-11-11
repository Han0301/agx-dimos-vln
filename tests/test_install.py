"""Exercise the bootstrap's non-mutating modes and SDK input boundaries."""
from pathlib import Path
import os
import subprocess
import tarfile
import io

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "sh/setup/install.sh"


@pytest.fixture
def workspace(tmp_path):
    (tmp_path / "sh/setup").mkdir(parents=True)
    (tmp_path / "dimos").mkdir()
    script = tmp_path / "sh/setup/install.sh"
    script.write_text(SOURCE.read_text())
    (tmp_path / "dimos/pyproject.toml").write_text("")
    (tmp_path / "dimos/requirements.lock").write_text("")
    return tmp_path


def invoke(root, *args):
    env = os.environ.copy()
    for name in ("CAMERA_SDK_DIR", "CAMERA_SDK_ARCHIVE", "CAMERA_SDK_URL", "CAMERA_SDK_SHA256"):
        env.pop(name, None)
    return subprocess.run(["bash", str(root / "sh/setup/install.sh"), *args],
                          env=env, text=True, capture_output=True, timeout=20)


def test_dry_run_does_not_create_environment_or_downloads(workspace):
    before = sorted(str(p.relative_to(workspace)) for p in workspace.rglob("*"))
    result = invoke(workspace, "--dry-run", "--camera", "all", "--recreate")
    assert result.returncode == 0, result.stderr
    assert "CameraSDK" in result.stdout
    assert "colcon build" in result.stdout
    assert "Environment verification" in result.stdout
    assert sorted(str(p.relative_to(workspace)) for p in workspace.rglob("*")) == before


def test_d455_plan_skips_proprietary_sdk_but_keeps_helpers(workspace):
    result = invoke(workspace, "--dry-run")
    assert result.returncode == 0, result.stderr
    assert "insta360_ros_driver" in result.stdout
    assert "BUILD_INSTA360_CAMERA=OFF" in result.stdout
    assert "== Insta360 SDK ==" not in result.stdout


@pytest.mark.parametrize("args", [
    ("--camera", "invalid"), ("--offline", "--sdk-url", "https://example.invalid/sdk"),
    ("--check-only", "--recreate"), ("--check-only", "--install-system-deps"),
    ("--check-only", "--dry-run"), ("--camera", "all", "--sdk-sha256", "invalid"), ("--unknown",),
    ("--sdk-dir", "/example/sdk"), ("--camera", "all", "--check-only", "--sdk-dir", "/example/sdk"),
])
def test_invalid_arguments_fail_before_mutation(workspace, args):
    result = invoke(workspace, *args)
    assert result.returncode != 0
    assert "ERROR:" in result.stderr
    assert not (workspace / ".venv").exists()
    assert not (workspace / "camera_ws").exists()


def functions(root):
    # Load the real functions without invoking the top-level installation flow.
    source = (root / "sh/setup/install.sh").read_text().rsplit("\npreflight\n", 1)[0]
    (root / "sh/setup/functions.sh").write_text(source)
    return f'source "{root}/sh/setup/functions.sh"\n'


def test_sdk_path_traversal_is_rejected(workspace):
    archive = workspace / "bad.tar"
    with tarfile.open(archive, "w") as handle:
        member = tarfile.TarInfo("../escaped")
        member.size = 1
        handle.addfile(member, io.BytesIO(b"x"))
    dest = workspace / "extract"
    dest.mkdir()
    result = subprocess.run(["bash", "-c", functions(workspace) +
                             f'extract_sdk "{archive}" "{dest}"'], text=True, capture_output=True)
    assert result.returncode != 0
    assert "Unsafe SDK archive path" in result.stderr
    assert not (workspace / "escaped").exists()


def test_sdk_symlink_archive_is_rejected(workspace):
    archive = workspace / "link.tar"
    with tarfile.open(archive, "w") as handle:
        member = tarfile.TarInfo("sdk/link")
        member.type = tarfile.SYMTYPE
        member.linkname = "/tmp"
        handle.addfile(member)
    dest = workspace / "extract"
    dest.mkdir()
    result = subprocess.run(["bash", "-c", functions(workspace) +
                             f'extract_sdk "{archive}" "{dest}"'], text=True, capture_output=True)
    assert result.returncode != 0
    assert "links/special files" in result.stderr


@pytest.mark.parametrize("system_packages", [True, False])
def test_venv_isolation_validation(workspace, system_packages):
    subprocess.run(["/usr/bin/python3.12", "-m", "venv", "--without-pip",
                    str(workspace / ".venv")], check=True)
    cfg = workspace / ".venv/pyvenv.cfg"
    if system_packages:
        cfg.write_text(cfg.read_text().replace("include-system-site-packages = false",
                                               "include-system-site-packages = true"))
    result = subprocess.run(["bash", "-c", functions(workspace) + "validate_venv"],
                            text=True, capture_output=True)
    assert (result.returncode != 0) == system_packages
    if system_packages:
        assert "sh/setup/install.sh" in result.stderr


def test_safe_sdk_archive_extracts(workspace):
    archive = workspace / "sdk.tar"
    with tarfile.open(archive, "w") as handle:
        member = tarfile.TarInfo("sdk/include/camera/camera.h")
        member.size = 1
        handle.addfile(member, io.BytesIO(b"x"))
    dest = workspace / "extract"
    dest.mkdir()
    result = subprocess.run(["bash", "-c", functions(workspace) +
                             f'extract_sdk "{archive}" "{dest}"'], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert (dest / "sdk/include/camera/camera.h").read_bytes() == b"x"


@pytest.mark.parametrize("status,log,success", [
    (0, "node finished normally", True),
    (0, "\x1b[31m[RTPS Error] Trying to open logical port 0", False),
    (124, "", False),
])
def test_lifecycle_checks_native_errors_and_timeout(workspace, status, log, success):
    (workspace / "lifecycle.log").write_text(log)
    result = subprocess.run(["bash", "-c", functions(workspace) +
                             f'temp="{workspace}"; trap - EXIT; '
                             f'export FASTRTPS_DEFAULT_PROFILES_FILE=/example/client_dds.xml; '
                             f'check_lifecycle_log {status}'], text=True, capture_output=True)
    assert (result.returncode == 0) == success


def test_default_installs_tracker_and_allows_explicit_trim(workspace):
    result = invoke(workspace, '--dry-run', '--camera', 'none', '--plugin', 'path_tracker')
    assert result.returncode == 0, result.stderr
    assert 'path_tracker' in result.stdout
    assert 'go2' in result.stdout
    assert not (workspace / '.venv').exists()
    base = invoke(workspace, '--dry-run', '--camera', 'none')
    assert base.returncode == 0, base.stderr
    assert 'path_tracker' in base.stdout
    trimmed = invoke(workspace, '--dry-run', '--camera', 'none', '--plugin', 'none')
    assert trimmed.returncode == 0, trimmed.stderr
    assert 'path_tracker' not in trimmed.stdout
    invalid = invoke(workspace, '--dry-run', '--plugin', 'missing')
    assert invalid.returncode != 0
    assert not (workspace / '.venv').exists()



def test_default_plan_includes_d455_body_and_tracker(workspace):
    result = invoke(workspace, '--dry-run')
    assert result.returncode == 0, result.stderr
    for name in ('realsense2_camera', 'compressed_RGBD_pkg', 'go2', 'path_tracker'):
        assert name in result.stdout


def test_incompatible_venv_is_selected_for_backup_without_extra_flags(workspace):
    (workspace / '.venv').mkdir()
    before = sorted(str(p.relative_to(workspace)) for p in workspace.rglob('*'))
    result = invoke(workspace, '--dry-run')
    assert result.returncode == 0, result.stderr
    assert '.venv.backup-' in result.stdout
    assert 'backed up and rebuilt' in result.stdout
    assert before == sorted(str(p.relative_to(workspace)) for p in workspace.rglob('*'))


def test_compatible_venv_is_reused_and_check_only_never_migrates(workspace):
    (workspace / '.venv').mkdir()
    code = functions(workspace) + '\nvalidate_venv() { return 0; }\nselect_venv_recreation\n[[ "$recreate" == 0 ]]\n'
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    code = functions(workspace) + '\ncheck_only=1\nvalidate_venv() { return 1; }\nselect_venv_recreation\n[[ "$recreate" == 0 ]]\n'
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not list(workspace.glob('.venv.backup-*'))


def test_check_only_does_not_install_missing_system_packages(workspace):
    marker = workspace / 'apt_was_called'
    code = functions(workspace) + f'\ncheck_only=1\ndpkg-query() {{ return 1; }}\nsudo() {{ touch "{marker}"; }}\napt-get() {{ touch "{marker}"; }}\nsystem_dependencies\n'
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True)
    assert result.returncode != 0
    assert not marker.exists()



def test_offline_mode_never_installs_system_packages(workspace):
    marker = workspace / 'apt_was_called'
    code = functions(workspace) + f'\noffline=1\ndpkg-query() {{ return 1; }}\nsudo() {{ touch "{marker}"; }}\napt-get() {{ touch "{marker}"; }}\nsystem_dependencies\n'
    result = subprocess.run(['bash', '-c', code], capture_output=True, text=True)
    assert result.returncode != 0
    assert not marker.exists()


@pytest.mark.parametrize("selection,expected", [
    ("agent", "go2,agent"), ("all", "go2,path_tracker,agent"),
])
def test_agent_dependencies_are_selectable_without_starting_models(workspace, selection, expected):
    result = invoke(workspace, "--dry-run", "--camera", "none", "--plugin", selection)
    assert result.returncode == 0, result.stderr
    import shlex
    install_line = next(line for line in result.stdout.splitlines() if "--constraint" in line)
    assert any(part.endswith(f"dimos[{expected}]") for part in shlex.split(install_line))
    assert not (workspace / ".venv").exists()

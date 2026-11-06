# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build-time pybind11 support for the A* extension; unused during navigation."""

from pathlib import Path
import struct
import sys

from pybind11.setup_helpers import Pybind11Extension, build_ext


def python_is_macos_universal_binary(executable: str | None = None) -> bool:
    """
    Returns True if the given executable is a macOS universal (fat) binary.
    """
    FAT_MAGIC = 0xCAFEBABE  # big-endian fat
    FAT_CIGAM = 0xBEBAFECA  # little-endian fat
    FAT_MAGIC_64 = 0xCAFEBABF  # big-endian fat 64
    FAT_CIGAM_64 = 0xBFBAFECA  # little-endian fat 64

    if executable is None:
        executable = sys.executable

    path = Path(executable)
    if not path.exists():
        return False

    try:
        with path.open("rb") as f:
            header = f.read(4)
            if len(header) < 4:
                return False

            magic = struct.unpack(">I", header)[0]
            return magic in {
                FAT_MAGIC,
                FAT_CIGAM,
                FAT_MAGIC_64,
                FAT_CIGAM_64,
            }
    except OSError:
        return False


class AStarBuildExt(build_ext):
    """Resolve pybind11 headers and CPU flags only when building extensions."""

    def finalize_options(self) -> None:
        extensions = []
        for extension in self.distribution.ext_modules or []:
            if extension.name == "dimos.navigation.replanning_a_star.min_cost_astar_ext":
                compile_args = list(extension.extra_compile_args or [])
                if not python_is_macos_universal_binary():
                    compile_args.append("-march=native")
                extension = Pybind11Extension(
                    extension.name,
                    extension.sources,
                    define_macros=[tuple(macro) for macro in extension.define_macros],
                    extra_compile_args=compile_args,
                )
            extensions.append(extension)
        self.distribution.ext_modules = extensions
        super().finalize_options()

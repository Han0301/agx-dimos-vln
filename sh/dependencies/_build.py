"""Make a self-contained source release from the workspace layout."""
from pathlib import Path
import shutil
import tomllib
from setuptools.command.sdist import sdist as _sdist


class sdist(_sdist):
    def make_release_tree(self, base_dir, files):
        local_files = [name for name in files
                       if not Path(name).is_absolute() and ".." not in Path(name).parts]
        super().make_release_tree(base_dir, local_files)
        metadata = Path(__file__).with_name("pyproject.toml")
        config = tomllib.loads(metadata.read_text(encoding="utf-8"))
        mapping = config["tool"]["setuptools"]["package-dir"]["dimos"]
        source = (metadata.parent / mapping).resolve()
        release = Path(base_dir)
        shutil.copytree(source, release / "dimos", dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.egg-info"))
        target = release / "pyproject.toml"
        content = target.read_text(encoding="utf-8").replace(
            'package-dir = {"dimos" = "../../dimos"}',
            'package-dir = {"dimos" = "dimos"}')
        # The release file may be hard-linked to the working metadata.
        target.unlink()
        target.write_text(content, encoding="utf-8")

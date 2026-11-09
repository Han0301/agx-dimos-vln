#!/usr/bin/env python3
"""Prepare the CLIP model in the data directory used by the active workspace."""
import hashlib
import os
from pathlib import Path
import shutil
import tarfile
import tempfile

from dimos.utils.data import _get_repo_root, get_data

repo = _get_repo_root()
target = repo / 'data' / 'models_clip'
model = target / 'model.onnx'
if model.is_file() and model.stat().st_size > 1024:
    print(f'CLIP model already installed: {model}')
    raise SystemExit(0)
pointer = repo / 'data' / '.lfs' / 'models_clip.tar.gz'
expected = None
if pointer.exists() and pointer.stat().st_size < 1024:
    for line in pointer.read_text().splitlines():
        if line.startswith('oid sha256:'):
            expected = line.split(':', 1)[1]
workspace = Path(__file__).resolve().parents[2]
custom = os.environ.get('DIMOS_CLIP_ARCHIVE')
candidates = ([Path(custom).expanduser()] if custom else []) + [
    workspace / 'data/.lfs/models_clip.tar.gz',
    workspace.parent / 'dimos/data/.lfs/models_clip.tar.gz',
    workspace.parent / 'dimos-vln/data/.lfs/models_clip.tar.gz',
]
for archive in candidates:
    if not expected or not archive.is_file() or archive.stat().st_size < 1024:
        continue
    digest = hashlib.sha256()
    with archive.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != expected:
        print(f'Ignoring CLIP archive with mismatched checksum: {archive}')
        continue
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.clip-install-', dir=target.parent) as temporary:
        with tarfile.open(archive, 'r:gz') as tar:
            tar.extractall(temporary, filter='data')
        extracted = Path(temporary) / 'models_clip'
        if not (extracted / 'model.onnx').is_file():
            raise RuntimeError(f'Invalid CLIP archive: {archive}')
        if target.exists():
            raise RuntimeError(f'Incomplete CLIP directory exists; inspect it: {target}')
        shutil.move(str(extracted), str(target))
    print(f'CLIP model installed from {archive} to {target}')
    break
else:
    if os.environ.get('DIMOS_CLIP_OFFLINE') == '1':
        raise RuntimeError('Offline mode: no matching local CLIP archive; set DIMOS_CLIP_ARCHIVE')
    print('No matching local CLIP archive; fetching using DimOS Git LFS...')
    get_data('models_clip')
if not model.is_file() or model.stat().st_size <= 1024:
    raise RuntimeError(f'CLIP model missing or invalid: {model}')
print(f'CLIP model ready: {model}')

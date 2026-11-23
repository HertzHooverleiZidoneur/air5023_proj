"""Fetch the official legacy RoboCasa assets at an immutable HF revision."""
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.cache/assets'
ASSETS = ROOT / 'third_party/stretch_mujoco/third_party/robocasa/robocasa/models/assets'
SENTINELS = {
    'fixtures.zip': 'fixtures/accessories/stools/rattan_stool/visuals/model_0.obj',
    'textures.zip': 'textures/tiles/concrete_tiles.png',
    'objaverse.zip': 'objects/objaverse/cup/cup_2/model.xml',
}


def fetch(item):
    name, metadata = item
    destination = CACHE / name
    marker = CACHE / (name + '.extracted')
    if marker.exists() and marker.read_text() == metadata['sha256'] and (ASSETS / SENTINELS[name]).exists():
        print(f'{name} already verified and extracted', flush=True)
        return
    url = f'https://huggingface.co/datasets/{MANIFEST["repository"]}/resolve/{MANIFEST["revision"]}/{name}'
    subprocess.run(['wget', '-c', '--timeout=60', '--tries=5', '--progress=dot:giga',
                    '-O', str(destination), url], check=True)
    digest = hashlib.sha256()
    with destination.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    if destination.stat().st_size != metadata['size'] or digest.hexdigest() != metadata['sha256']:
        raise RuntimeError(f'Asset verification failed: {destination}')
    target = ASSETS / 'objects' if name == 'objaverse.zip' else ASSETS
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination) as archive:
        for entry in archive.infolist():
            if not (target / entry.filename).resolve().is_relative_to(target.resolve()):
                raise ValueError(f'Unsafe archive path: {entry.filename}')
        for entry in archive.infolist():
            # Upstream code already supplies some MJCF files. Asset bundles
            # must not overwrite those tracked files with newer versions.
            if not (target / entry.filename).exists():
                archive.extract(entry, target)
    marker.write_text(metadata['sha256'])
    print(f'{name} verified and extracted', flush=True)


if __name__ == '__main__':
    MANIFEST = json.loads((ROOT / 'configs/assets.lock.json').read_text())
    CACHE.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(fetch, MANIFEST['files'].items()))

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PROJECT_ROOT="$PWD"
export UV_CACHE_DIR="$PROJECT_ROOT/.cache/uv"
export UV_PYTHON_INSTALL_DIR="$PROJECT_ROOT/.tools/python"
export NUMBA_CACHE_DIR="$PROJECT_ROOT/.cache/numba"
export MPLCONFIGDIR="$PROJECT_ROOT/.cache/matplotlib"
mkdir -p .tools/packages .cache third_party

for executable in python3 wget gcc; do
    command -v "$executable" >/dev/null || { echo "Missing prerequisite: $executable" >&2; exit 1; }
done
if ! command -v git >/dev/null; then
    if [[ ! -x .tools/usr/bin/git ]]; then
        # Ubuntu 22.04 fallback, unpack locally without changing system packages.
        command -v apt-get >/dev/null && command -v dpkg-deb >/dev/null
        (cd .tools/packages && apt-get download 'git=1:2.34.1-1ubuntu1.18')
        dpkg-deb -x .tools/packages/git_*_amd64.deb .tools
    fi
    export PATH="$PROJECT_ROOT/.tools/usr/bin:$PATH"
    export GIT_EXEC_PATH="$PROJECT_ROOT/.tools/usr/lib/git-core"
fi
if [[ ! -x .tools/uv ]]; then
    python3 - <<'PY'
import hashlib, io, pathlib, urllib.request, zipfile
url = 'https://files.pythonhosted.org/packages/73/da/2597becbc0fcbb59608d38fda5db79969e76dedf5b072f0e8564c8f0628b/uv-0.9.11-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl'
payload = urllib.request.urlopen(url, timeout=60).read()
assert hashlib.sha256(payload).hexdigest() == 'e97906ca1b90dac91c23af20e282e2e37c8eb80c3721898733928a295f2defda'
with zipfile.ZipFile(io.BytesIO(payload)) as z:
    member = next(n for n in z.namelist() if n.endswith('/uv'))
    pathlib.Path('.tools/uv').write_bytes(z.read(member))
pathlib.Path('.tools/uv').chmod(0o755)
PY
fi

STRETCH_COMMIT=c78d6a1cd79bb5ead1b679610299563c1a91124a
if [[ ! -d third_party/stretch_mujoco/.git ]]; then
    git clone https://github.com/hello-robot/stretch_mujoco.git third_party/stretch_mujoco
fi
if [[ -n "$(git -C third_party/stretch_mujoco diff --name-only)" ]]; then
    echo 'Third-party tracked files have local edits. Preserve and resolve them before installing.' >&2
    exit 1
fi
git -C third_party/stretch_mujoco checkout "$STRETCH_COMMIT"
git -C third_party/stretch_mujoco submodule update --init --recursive
.tools/uv python install 3.10.19
if [[ ! -x .venv/bin/python ]]; then
    .tools/uv venv --python "$UV_PYTHON_INSTALL_DIR/cpython-3.10.19-linux-x86_64-gnu/bin/python3.10" .venv
fi
.venv/bin/python -c 'import sys; assert sys.version_info[:3] == (3,10,19), "Expected Python 3.10.19"'
.tools/uv pip sync --python .venv/bin/python requirements.lock \
    --extra-index-url https://download.pytorch.org/whl/cpu --index-strategy unsafe-best-match
.tools/uv pip check --python .venv/bin/python
if [[ "${1:-}" != --skip-assets ]]; then
    .venv/bin/python 03_data_generate/download_assets.py
    .venv/bin/python 03_data_generate/build_scene.py
fi
echo 'Installation complete. Run .venv/bin/python 02_manipulation_vla/run_task.py'

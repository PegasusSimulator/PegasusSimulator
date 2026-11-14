#!/bin/bash
# Build QGroundControl from source (needed on Ubuntu 22.04: the prebuilt AppImage requires glibc >= 2.38).
# Follows https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/
# Usage: ./build_qgc.sh [git-ref]        (default: latest release tag)
# Result: ~/qgroundcontrol/build/Release/QGroundControl  (+ symlink ~/QGroundControl-built)

set -Eeo pipefail
export DEBIAN_FRONTEND=noninteractive GIT_TERMINAL_PROMPT=0

QGC_SRC="${QGC_SRC:-$HOME/qgroundcontrol}"
QGC_REF="${1:-}"
JOBS="${JOBS:-$(nproc)}"

trap 'echo "!!! build_qgc.sh failed at line $LINENO: $BASH_COMMAND"; echo "!!! Re-run to resume (clone/Qt/build steps are incremental)."' ERR

[ "$(id -u)" -ne 0 ] || { echo "Do not run as root."; exit 1; }
avail=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
[ "${avail:-0}" -ge 25 ] || { echo "Need >= 25GB free in \$HOME (Qt ~2.2GB + build), have ${avail}GB"; exit 1; }
curl -sfI --max-time 15 https://github.com >/dev/null || { echo "No access to github.com"; exit 1; }

if [ -x "$HOME/QGroundControl-built" ] && [ -z "$QGC_REF" ] && [ -z "$FORCE" ]; then
    echo "QGC already built (set FORCE=1 to rebuild)."; exit 0
fi

echo ">>> [1/5] Build tools (pip cmake >= 3.25, ninja; Ubuntu 22.04 ships CMake 3.22)"
sudo apt-get update -qq
sudo apt-get install -y git curl python3-pip ninja-build libxcb-cursor0 ca-certificates
# Settle the CA bundle *before* any Python download (an apt run that rebuilds it mid-download
# makes urllib fail with CERTIFICATE_VERIFY_FAILED)
sudo update-ca-certificates >/dev/null
py_https_ok() {
    python3 -c "import urllib.request
for u in ('https://github.com', 'https://release-assets.githubusercontent.com'):
    try: urllib.request.urlopen(u, timeout=15)
    except urllib.error.HTTPError: pass   # any HTTP status means TLS verified fine" 2>/dev/null
}
if ! py_https_ok; then
    echo "    Python failed TLS verification with the system CA store. Diagnostics:"
    env | grep -iE '^(ssl_|requests_ca|curl_ca|https?_proxy|no_proxy)' | sed 's/^/      /' || true
    python3 -c "import ssl; print('      verify paths:', ssl.get_default_verify_paths())"
    echo "    Falling back to the certifi (Mozilla) CA bundle for this build..."
    unset SSL_CERT_DIR
    python3 -m pip install --user --quiet --upgrade certifi
    export SSL_CERT_FILE="$(python3 -m certifi)"
    export REQUESTS_CA_BUNDLE="$SSL_CERT_FILE"
    py_https_ok || { echo "ERROR: Python still cannot verify HTTPS (proxy/firewall doing TLS interception?)"; exit 1; }
    echo "    OK using $SSL_CERT_FILE"
fi
export PATH="$HOME/.local/bin:$PATH"
python3 -m pip install --user --quiet --upgrade cmake
verify_cmake() { cmake --version | head -1; }
verify_cmake

echo ">>> [2/5] Clone QGroundControl"
if [ ! -d "$QGC_SRC/.git" ]; then
    git clone -j8 https://github.com/mavlink/qgroundcontrol.git "$QGC_SRC"
fi
cd "$QGC_SRC"
git fetch --tags --quiet
if [ -z "$QGC_REF" ]; then
    QGC_REF=$(git tag --sort=-v:refname | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' | head -1)
fi
echo "    Using ref: $QGC_REF"
git checkout --quiet "$QGC_REF"
git submodule update --init --recursive

# Tooling differs between QGC versions, so read everything from the checked-out repo
QT_VER=$(python3 ./tools/setup/read_config.py --get qt.version)
QT_MODULES=$(python3 ./tools/setup/read_config.py --get qt.modules)
QT_ROOT="$HOME/Qt/$QT_VER/gcc_64"
echo "    Qt version required by this QGC: $QT_VER"

# Repo .venv built by QGC's own bootstrap: seeds pip (CMake's MAVLink step needs it) and the
# generator deps CMake checks for (a hand-made `uv venv` gets re-synced by CMake, which drops pip).
ensure_venv() {
    if ! .venv/bin/python -m pip --version &>/dev/null; then
        rm -rf .venv
        UV_LINK_MODE=copy python3 tools/setup/install_python.py scripts
    fi
    .venv/bin/python -m pip --version &>/dev/null || { echo "ERROR: $QGC_SRC/.venv has no pip"; exit 1; }
}

echo ">>> [3/5] System dependencies (apt, uses sudo)"
python3 ./tools/setup/install_dependencies --platform debian || {
    echo "    Dependency step failed — retrying once (transient network/CA-bundle errors)..."
    sleep 5
    python3 ./tools/setup/install_dependencies --platform debian
}

echo ">>> [4/5] Qt $QT_VER (installs to ~/Qt, ~2.2GB)"
if [ -x "$QT_ROOT/bin/qt-cmake" ]; then
    echo "    Qt already installed at $QT_ROOT"
else
    # QGC's pip_install() falls back to `uv pip install --system` (-> /usr/local, permission denied)
    # unless a repo .venv exists. Give it one, and put it first on PATH so `aqt` is found there.
    ensure_venv
    export PATH="$QGC_SRC/.venv/bin:$PATH" UV_LINK_MODE=copy
    .venv/bin/python ./tools/setup/install_qt.py install --version "$QT_VER" --host linux --target desktop \
        --arch linux_gcc_64 --outdir "$HOME/Qt" --modules "$QT_MODULES"
fi
[ -x "$QT_ROOT/bin/qt-cmake" ] || { echo "ERROR: Qt install incomplete (no $QT_ROOT/bin/qt-cmake)"; exit 1; }

echo ">>> [5/5] Configure + build (Release, $JOBS jobs; this takes a while)"
# Venv must exist with pip even when the Qt step was skipped (Qt already installed)
export PATH="$QGC_SRC/.venv/bin:$PATH" UV_LINK_MODE=copy
ensure_venv
python3 ./tools/configure.py -B build --release --qt-root "$QT_ROOT"
cmake --build build --config Release --parallel "$JOBS"

OUT_BIN=$(find build -maxdepth 3 -type f -name QGroundControl -perm -u+x | head -1)
[ -n "$OUT_BIN" ] || { echo "ERROR: build finished but QGroundControl binary not found under $QGC_SRC/build"; exit 1; }
OUT_BIN="$QGC_SRC/$OUT_BIN"
ln -sf "$OUT_BIN" "$HOME/QGroundControl-built"
echo ""
echo "QGC built: $OUT_BIN"
echo "Run with:  ~/QGroundControl-built"

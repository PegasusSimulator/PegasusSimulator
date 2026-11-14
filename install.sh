#!/bin/bash
# PegasusSimulator Full Installation Script
# Installs: Isaac Sim 5.1.0, PegasusSimulator, PX4 v1.14.3, QGC, uXRCE-DDS
# Tested on Ubuntu 22.04 LTS with NVIDIA driver >= 550

set -Eeo pipefail  # Exit on error (incl. inside pipes/functions)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$HOME/pegasus_install.log"
exec > >(tee -a "$LOG_FILE") 2>&1

export DEBIAN_FRONTEND=noninteractive   # apt never blocks on prompts
export GIT_TERMINAL_PROMPT=0            # git never blocks asking for credentials

# ═════════════ SAFEGUARDS ═════════════
CURRENT_STEP="startup"
step() { CURRENT_STEP="$1"; echo ""; echo ">>> $1"; }

on_error() {
    local code=$? line=${BASH_LINENO[0]}
    echo ""
    echo "!!! FAILED during: $CURRENT_STEP"
    echo "!!! Exit code $code at install.sh line $line: $BASH_COMMAND"
    echo "!!! Fix the cause and re-run ./install.sh — completed steps are verified and skipped."
    echo "!!! Full log: $LOG_FILE"
}
trap on_error ERR
trap 'echo ""; echo "!!! Interrupted during: $CURRENT_STEP (re-run to resume)"; exit 130' INT TERM

# need_space <path> <GB>: abort early if the disk is too full
need_space() {
    local avail_gb
    avail_gb=$(df -BG --output=avail "$1" | tail -1 | tr -dc '0-9')
    if [ "${avail_gb:-0}" -lt "$2" ]; then
        echo "ERROR: need >= ${2}GB free on $1, only ${avail_gb}GB available."
        exit 1
    fi
}

# safe_download <url> <dest> <min_bytes> [resume]
# Downloads to a .part file, retries, checks size, and only then moves into place,
# so a failed/partial download can never be mistaken for a finished one.
safe_download() {
    local url="$1" dest="$2" min="$3" resume="${4:-}" part="$2.part" size
    [ -z "$resume" ] && rm -f "$part"
    mkdir -p "$(dirname "$dest")"
    wget ${resume:+-c} -L --tries=3 --timeout=30 --show-progress -O "$part" "$url" \
        || { echo "ERROR: download failed: $url"; rm -f "${part}"; return 1; }
    size=$(stat -c %s "$part" 2>/dev/null || echo 0)
    if [ "$size" -lt "$min" ]; then
        echo "ERROR: $url returned only $size bytes (expected >= $min) — wrong URL or error page."
        rm -f "$part"; return 1
    fi
    mv -f "$part" "$dest"
}

# verify <description> <command...>: fail loudly if a component isn't actually usable
verify() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then echo "    [OK]   $desc"
    else echo "    [FAIL] $desc"; return 1; fi
}

echo "============================================================"
echo "  PegasusSimulator Installation — $(date)"
echo "  Log: $LOG_FILE"
echo "============================================================"

# ─────────────────────────────────────────────
# STEP 0: Verify Prerequisites
# ─────────────────────────────────────────────
step "[0/6] Checking prerequisites..."

[ "$(id -u)" -eq 0 ] && { echo "ERROR: do not run as root (run as your user; sudo is used where needed)."; exit 1; }
[ "$(uname -m)" = "x86_64" ] || { echo "ERROR: this script supports x86_64 only."; exit 1; }
for c in wget curl unzip git sudo; do
    command -v "$c" &>/dev/null || { echo "Installing missing tool: $c"; sudo apt-get install -y "$c"; }
done
curl -sfI --max-time 15 https://github.com >/dev/null || { echo "ERROR: no internet access to github.com."; exit 1; }
sudo -v || { echo "ERROR: sudo authentication failed."; exit 1; }
need_space "$HOME" 60   # Isaac Sim zip + extracted + PX4 build

if ! command -v nvidia-smi &>/dev/null; then
    echo "ERROR: NVIDIA driver not found. Install driver >= 550 first."
    exit 1
fi

DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
echo "    NVIDIA driver: $DRIVER_VER"

UBUNTU_VER=$(grep "^VERSION_ID=" /etc/os-release | cut -d'"' -f2)
if [[ "$UBUNTU_VER" != "22.04" && "$UBUNTU_VER" != "24.04" ]]; then
    echo "WARNING: Ubuntu $UBUNTU_VER is not officially tested (22.04/24.04 recommended)."
fi
echo "    Ubuntu: $UBUNTU_VER"
echo "    Architecture: $(uname -m)"
echo ""

# ─────────────────────────────────────────────
# STEP 1: Install Isaac Sim 5.1.0
# ─────────────────────────────────────────────
step "[1/6] Installing Isaac Sim 5.1.0..."

ISAACSIM_PATH="$HOME/isaacsim"
ISAACSIM_ZIP="isaac-sim-standalone-5.1.0-linux-x86_64.zip"
ISAACSIM_URL="https://download.isaacsim.omniverse.nvidia.com/$ISAACSIM_ZIP"

if [ -x "$ISAACSIM_PATH/isaac-sim.sh" ] && [ -x "$ISAACSIM_PATH/python.sh" ]; then
    echo "    Isaac Sim already installed at $ISAACSIM_PATH — skipping download."
else
    mkdir -p "$ISAACSIM_PATH"
    cd "$ISAACSIM_PATH"

    echo "    Downloading Isaac Sim (~10 GB) — this may take a while..."
    # resumable (-c); only renamed to the final .zip once complete and >= 5GB
    safe_download "$ISAACSIM_URL" "$ISAACSIM_ZIP" 5000000000 resume

    echo "    Verifying zip integrity..."
    unzip -tq "$ISAACSIM_ZIP" >/dev/null || { echo "ERROR: corrupt zip — deleting; re-run to re-download."; rm -f "$ISAACSIM_ZIP"; exit 1; }

    echo "    Unzipping..."
    unzip -oq "$ISAACSIM_ZIP"

    echo "    Running post-install script..."
    chmod +x ./post_install.sh
    ./post_install.sh

    echo "    Cleaning up zip..."
    rm -f "$ISAACSIM_ZIP"

    echo "    Isaac Sim 5.1.0 installed at $ISAACSIM_PATH"
fi
cd "$HOME"
verify "Isaac Sim launcher + python.sh present" test -x "$ISAACSIM_PATH/isaac-sim.sh" -a -x "$ISAACSIM_PATH/python.sh"

# ─────────────────────────────────────────────
# STEP 2: Configure ~/.bashrc for Isaac Sim
# ─────────────────────────────────────────────
echo ""
echo ">>> [2/6] Configuring environment (~/.bashrc)..."

BASHRC="$HOME/.bashrc"

if grep -q "ISAACSIM_PATH" "$BASHRC"; then
    echo "    Isaac Sim env vars already in ~/.bashrc — skipping."
else
    cat >> "$BASHRC" << 'BASHRC_BLOCK'

# ── Isaac Sim ──────────────────────────────────────────────────
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

isaac_run() {
    if [ ! -x "$ISAACSIM_PYTHON" ]; then
        echo "Isaac Sim python.sh not found at: $ISAACSIM_PYTHON"
        return 1
    fi
    if [ ! -x "$ISAACSIM" ]; then
        echo "Isaac Sim launcher not found at: $ISAACSIM"
        return 1
    fi

    # Unset ROS 2 env to avoid conflicts with Isaac's Python 3.11
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH \
          COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH

    # Strip ROS 2 paths from LD_LIBRARY_PATH
    local ros_paths=("/opt/ros/humble" "/opt/ros/jazzy" "/opt/ros/iron")
    for ros_path in "${ros_paths[@]}"; do
        export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' \
            | grep -v "^${ros_path}" | paste -sd':' -)
    done

    # Detect Ubuntu version and set matching ROS2 bridge
    if [ -f /etc/os-release ]; then
        UBUNTU_VERSION=$(grep "^VERSION_ID=" /etc/os-release | cut -d'"' -f2)
    fi

    if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
        export ROS_DISTRO=jazzy
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/jazzy/lib"
        echo "Detected Ubuntu 24.04 -> ROS_DISTRO=jazzy"
    else
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
        echo "Detected Ubuntu ${UBUNTU_VERSION:-unknown} -> ROS_DISTRO=humble"
    fi

    if [ $# -eq 0 ]; then
        echo "Launching Isaac Sim GUI..."
        "${ISAACSIM}"
    elif [[ "$1" == --* ]]; then
        echo "Launching Isaac Sim with options: $*"
        "${ISAACSIM}" "$@"
    elif [ -f "$1" ]; then
        local SCRIPT_PATH="$1"
        shift
        echo "Running Python script with Isaac Sim: $SCRIPT_PATH"
        "${ISAACSIM_PYTHON}" "$SCRIPT_PATH" "$@"
    else
        echo "Unknown argument or file not found: '$1'"
        echo "Usage:"
        echo "  isaac_run                  -> launch GUI"
        echo "  isaac_run my_script.py     -> run script with IsaacSim Python"
        echo "  isaac_run --headless ...   -> launch IsaacSim with CLI flags"
        return 1
    fi
}
# ───────────────────────────────────────────────────────────────
BASHRC_BLOCK

    echo "    Isaac Sim environment added to ~/.bashrc"
fi

# Source so variables are available in this script session
export ISAACSIM_PATH="$HOME/isaacsim"
export ISAACSIM_PYTHON="$ISAACSIM_PATH/python.sh"
export ISAACSIM="$ISAACSIM_PATH/isaac-sim.sh"

# ─────────────────────────────────────────────
# STEP 3: Install PegasusSimulator as Python library
# ─────────────────────────────────────────────
step "[3/6] Installing PegasusSimulator Python extension..."

PEGASUS_DIR="$SCRIPT_DIR"

if [ ! -f "$PEGASUS_DIR/extensions/pegasus.simulator/setup.py" ] && \
   [ ! -f "$PEGASUS_DIR/extensions/pegasus.simulator/pyproject.toml" ]; then
    echo "    Cannot find pegasus.simulator extension at $PEGASUS_DIR/extensions/"
    echo "    Make sure you are running this script from the PegasusSimulator repo root."
    exit 1
fi

if "$ISAACSIM_PYTHON" -c "import pegasus.simulator" 2>/dev/null; then
    echo "    pegasus.simulator already importable — skipping pip install."
else
    echo "    Installing pegasus.simulator into Isaac Sim Python..."
    "$ISAACSIM_PYTHON" -m pip install --editable "$PEGASUS_DIR/extensions/pegasus.simulator"
    echo "    PegasusSimulator installed."
fi
verify "pegasus.simulator importable from Isaac Sim Python" "$ISAACSIM_PYTHON" -c "import pegasus.simulator"

# Link ./app -> Isaac Sim; self-heal a corrupt packman cache (incomplete download -> ModuleNotFoundError)
if [ ! -e "$PEGASUS_DIR/app" ]; then
    echo "    Linking Isaac Sim app (link_app.sh)..."
    if ! "$PEGASUS_DIR/link_app.sh" --path "$ISAACSIM_PATH"; then
        echo "    link_app.sh failed — clearing packman cache and retrying once..."
        rm -rf "$HOME"/packman-repo/packman-common
        "$PEGASUS_DIR/link_app.sh" --path "$ISAACSIM_PATH"
    fi
fi

# ─────────────────────────────────────────────
# STEP 4: Install PX4-Autopilot v1.14.3
# ─────────────────────────────────────────────
step "[4/6] Installing PX4-Autopilot v1.14.3..."

PX4_DIR="$HOME/PX4-Autopilot"

echo "    Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y git make cmake python3-pip

echo "    Installing Python dependencies..."
pip3 install --quiet kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

if [ -d "$PX4_DIR/.git" ]; then
    echo "    PX4-Autopilot repo already exists at $PX4_DIR — skipping clone."
    cd "$PX4_DIR"
else
    echo "    Cloning PX4-Autopilot..."
    git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
    cd "$PX4_DIR"
fi

# Checkout correct version and update submodules
CURRENT_TAG=$(git describe --tags 2>/dev/null || echo "unknown")
if [[ "$CURRENT_TAG" != "v1.14.3" ]]; then
    echo "    Checking out v1.14.3..."
    git fetch --tags
    git checkout v1.14.3
    echo "    Updating submodules (this may take a while)..."
    git submodule update --init --recursive
else
    echo "    Already on v1.14.3"
fi

# Build PX4 SITL
if [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "    PX4 SITL already built — skipping make."
else
    echo "    Building PX4 SITL (this may take 10-20 minutes)..."
    # Install PX4 Ubuntu dependencies
    bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
    # DONT_RUN=1 builds without launching the PX4 shell (make target 'none' otherwise starts PX4 SITL interactively)
    # </dev/null + timeout: even if PX4 ever launches, it cannot hang the installer
    DONT_RUN=1 timeout 90m make px4_sitl_default none </dev/null
    echo "    PX4 build complete."
fi
verify "PX4 SITL binary built (v1.14.3)" test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"

cd "$HOME"

# ─────────────────────────────────────────────
# STEP 5: Install QGroundControl
# ─────────────────────────────────────────────
step "[5/6] Installing QGroundControl (QGC)..."

GLIBC_VER=$(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$')
QGC_BUILT="$HOME/QGroundControl-built"

if [ "$(printf '%s\n' 2.38 "$GLIBC_VER" | sort -V | head -1)" != "2.38" ]; then
    # glibc < 2.38 (Ubuntu 22.04): the prebuilt AppImage cannot run -> build from source
    echo "    glibc $GLIBC_VER < 2.38: prebuilt AppImage unsupported, building QGC from source..."
    if [ -x "$QGC_BUILT" ]; then
        echo "    Source build already present at $QGC_BUILT — skipping."
    else
        "$SCRIPT_DIR/build_qgc.sh"
    fi
    verify "QGC source build present and executable" test -x "$QGC_BUILT"
    QGC_LAUNCH="$QGC_BUILT"
else
    QGC_APPIMAGE="$HOME/QGroundControl.AppImage"
    QGC_URL="https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage"

    # Remove leftovers from earlier failed runs (0-byte / non-ELF files)
    [ -e "$QGC_APPIMAGE" ] && ! head -c4 "$QGC_APPIMAGE" 2>/dev/null | grep -q "ELF" && { echo "    Removing invalid QGC file."; rm -f "$QGC_APPIMAGE"; }

    if [ -s "$QGC_APPIMAGE" ]; then
        echo "    QGroundControl already exists at $QGC_APPIMAGE — skipping download."
    else
        echo "    Installing QGC dependencies..."
        sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
                                libfuse2 libxcb-xinerama0 libxkbcommon-x11-0
        echo "    Downloading QGroundControl AppImage..."
        safe_download "$QGC_URL" "$QGC_APPIMAGE" 50000000   # real AppImage is >100MB
        chmod +x "$QGC_APPIMAGE"
    fi
    verify "QGC AppImage is executable ELF" bash -c "test -x '$QGC_APPIMAGE' && head -c4 '$QGC_APPIMAGE' | grep -q ELF"
    verify "QGC AppImage runtime responds" timeout 20 "$QGC_APPIMAGE" --appimage-version
    QGC_LAUNCH="$QGC_APPIMAGE"
fi

# ─────────────────────────────────────────────
# STEP 6: Install Micro XRCE-DDS Agent (uXRCE-DDS)
# ─────────────────────────────────────────────
step "[6/6] Installing Micro XRCE-DDS Agent (uXRCE-DDS)..."

XRCE_DIR="$HOME/Micro-XRCE-DDS-Agent"

if command -v MicroXRCEAgent &>/dev/null; then
    echo "    MicroXRCEAgent already in PATH — skipping."
else
    echo "    Installing build dependencies..."
    sudo apt-get install -y cmake g++ python3 python3-pip

    if [ -d "$XRCE_DIR/.git" ]; then
        echo "    Micro-XRCE-DDS-Agent repo already exists — skipping clone."
    else
        echo "    Cloning Micro-XRCE-DDS-Agent..."
        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$XRCE_DIR"
    fi

    cd "$XRCE_DIR"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/

    # Verify install
    if command -v MicroXRCEAgent &>/dev/null; then
        echo "    MicroXRCEAgent installed: $(MicroXRCEAgent --version 2>/dev/null || echo 'ok')"
    else
        echo "    WARNING: MicroXRCEAgent not found in PATH after install."
        echo "    Try: export PATH=\$PATH:/usr/local/bin"
    fi
    cd "$HOME"
fi
verify "MicroXRCEAgent in PATH" command -v MicroXRCEAgent

# ─────────────────────────────────────────────
# DONE
# ─────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  Installation Complete!"
echo "============================================================"
echo ""
echo "  Components installed:"
echo "    Isaac Sim 5.1.0    ->  $HOME/isaacsim/"
echo "    PegasusSimulator   ->  $SCRIPT_DIR/extensions/pegasus.simulator"
echo "    PX4-Autopilot      ->  $HOME/PX4-Autopilot/"
echo "    QGroundControl     ->  $QGC_LAUNCH"
echo "    uXRCE-DDS Agent    ->  /usr/local/bin/MicroXRCEAgent"
echo ""
echo "  Next steps:"
echo "    1. Reload your shell:  source ~/.bashrc"
echo "    2. Launch Isaac Sim:   isaac_run"
echo "    3. Enable the Pegasus extension in Isaac Sim:"
echo "       Window -> Extensions -> (add path: $SCRIPT_DIR/extensions)"
echo "    4. Start uXRCE-DDS:    MicroXRCEAgent udp4 -p 8888"
echo "    5. Launch QGC:         $QGC_LAUNCH"
echo ""
echo "  Full install log saved to: $LOG_FILE"
